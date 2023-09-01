use std::{
    io::{self, Stdout},
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::{Context, Result};
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use enumflags2::{bitflags, BitFlags};
use num_format::{SystemLocale, ToFormattedString};
use ratatui::{prelude::*, widgets::*};

use crate::ih_modbus::InductionHeaterState;

pub fn run_tui(state: Arc<Mutex<InductionHeaterState>>) -> Result<()> {
    let mut terminal = setup_terminal().context("setup failed")?;
    run(&mut terminal, state).context("app loop failed")?;
    restore_terminal(&mut terminal).context("restore terminal failed")?;
    std::process::exit(0);
}

/// Setup the terminal. This is where you would enable raw mode, enter the alternate screen, and
/// hide the cursor. This example does not handle errors. A more robust application would probably
/// want to handle errors and ensure that the terminal is restored to a sane state before exiting.
fn setup_terminal() -> Result<Terminal<CrosstermBackend<Stdout>>> {
    let mut stdout = io::stdout();
    enable_raw_mode().context("failed to enable raw mode")?;
    execute!(stdout, EnterAlternateScreen).context("unable to enter alternate screen")?;
    Terminal::new(CrosstermBackend::new(stdout)).context("creating terminal failed")
}

/// Restore the terminal. This is where you disable raw mode, leave the alternate screen, and show
/// the cursor.
fn restore_terminal(terminal: &mut Terminal<CrosstermBackend<Stdout>>) -> Result<()> {
    disable_raw_mode().context("failed to disable raw mode")?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)
        .context("unable to switch to main screen")?;
    terminal.show_cursor().context("unable to show cursor")
}

/// Run the application loop. This is where you would handle events and update the application
/// state. This example exits when the user presses 'q'. Other styles of application loops are
/// possible, for example, you could have multiple application states and switch between them based
/// on events, or you could have a single application state and update it based on events.
fn run(
    terminal: &mut Terminal<CrosstermBackend<Stdout>>,
    state: Arc<Mutex<InductionHeaterState>>,
) -> Result<()> {
    let mut graph_selection = BitFlags::empty();

    loop {
        terminal.draw(|frame| render_app(frame, &state, &graph_selection))?;
        if event::poll(Duration::from_millis(16)).context("event poll failed")? {
            match event::read().context("event read failed")? {
                Event::Key(key) => match key.code {
                    KeyCode::Char('q') => {
                        break;
                    }
                    KeyCode::Char('e') if key.kind == KeyEventKind::Press => {
                        let mut state_guard = state.lock().unwrap();
                        let flipped_target = !state_guard.target_enabled();
                        state_guard.set_target_enabled(flipped_target);
                    }
                    KeyCode::Char(c) if c.is_ascii_digit() && key.kind == KeyEventKind::Press => {
                        let num: u8 = c.to_digit(10).unwrap() as u8;
                        graph_selection.toggle(BitFlags::from_bits_truncate(1 << (num - 1)));
                    }
                    _ => {}
                },
                _ => (),
            }
        }
    }
    Ok(())
}

/// Render the application. This is where you would draw the application UI. This example just
/// draws a greeting.
fn render_app(
    frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>,
    state: &Mutex<InductionHeaterState>,
    current_selection: &BitFlags<GraphSelection>,
) {
    let state = state.lock().unwrap().clone();

    let top_bottom = Layout::default()
        .direction(Direction::Vertical)
        .constraints(
            [
                Constraint::Percentage(100),
                Constraint::Min(3),
                Constraint::Min(if state.error().is_some() { 1 } else { 0 }),
            ]
            .as_ref(),
        )
        .split(frame.size());

    let main_window = top_bottom[0];
    let keybindings_window = top_bottom[1];
    let error_window = top_bottom[2];

    let graph_active = !current_selection.is_empty();

    // Create two chunks with equal horizontal screen space
    let main_left_right = Layout::default()
        .direction(Direction::Horizontal)
        .constraints(if graph_active {
            [Constraint::Min(50), Constraint::Percentage(100)]
        } else {
            [Constraint::Percentage(100), Constraint::Percentage(0)]
        })
        .split(main_window);

    render_table_state(frame, main_left_right[0], &state, current_selection);
    if graph_active {
        render_graph(frame, main_left_right[1], &state, current_selection);
    }
    render_keybindings(frame, keybindings_window);
    if state.error().is_some() {
        render_error(frame, error_window, &state);
    }
}

fn render_table_state(
    frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>,
    area: Rect,
    state: &InductionHeaterState,
    current_selection: &BitFlags<GraphSelection>,
) {
    let rows = vec![
        Row::new::<Vec<Cell>>(vec![
            make_table_selection(BitFlags::empty(), current_selection).into(),
            Span::raw("Enabled:").into(),
            {
                let enabled_text = if state.enabled() {
                    Span::styled(
                        "YES",
                        Style::default()
                            .add_modifier(Modifier::ITALIC)
                            .fg(Color::White),
                    )
                } else {
                    Span::styled(
                        "NO",
                        Style::default()
                            .add_modifier(Modifier::ITALIC)
                            .fg(Color::DarkGray),
                    )
                };
                let target_text = if state.target_enabled() {
                    Span::styled(
                        "YES",
                        Style::default()
                            .add_modifier(Modifier::ITALIC)
                            .fg(Color::White),
                    )
                } else {
                    Span::styled(
                        "NO",
                        Style::default()
                            .add_modifier(Modifier::ITALIC)
                            .fg(Color::DarkGray),
                    )
                };

                Line::from(vec![
                    enabled_text,
                    Span::raw(" (target = "),
                    target_text,
                    Span::raw(")"),
                ])
                .into()
            },
        ]),
        Row::new::<Vec<Cell>>(vec![
            make_table_selection(GraphSelection::CoilDriveFrequency.into(), current_selection)
                .into(),
            Span::raw("Coil drive frequency:").into(),
            Span::raw(format!("{}", state.coil_drive_frequency())).into(),
        ]),
        Row::new::<Vec<Cell>>(vec![
            make_table_selection(GraphSelection::CoilVoltageMax.into(), current_selection).into(),
            Span::raw("Coil voltage max:").into(),
            Span::raw(format!("{}", state.coil_voltage_max())).into(),
        ]),
        Row::new::<Vec<Cell>>(vec![
            make_table_selection(GraphSelection::FanRpm.into(), current_selection).into(),
            Span::raw("Fan RPM:").into(),
            Span::raw(format!("{}", state.fan_rpm())).into(),
        ]),
        Row::new::<Vec<Cell>>(vec![
            make_table_selection(BitFlags::empty(), current_selection).into(),
            Span::raw("LED green:").into(),
            if state.led_green() {
                Span::styled(
                    "ON",
                    Style::default()
                        .add_modifier(Modifier::ITALIC)
                        .fg(Color::Green),
                )
            } else {
                Span::styled(
                    "OFF",
                    Style::default()
                        .add_modifier(Modifier::ITALIC)
                        .fg(Color::DarkGray),
                )
            }
            .into(),
        ]),
        Row::new::<Vec<Cell>>(vec![
            make_table_selection(BitFlags::empty(), current_selection).into(),
            Span::raw("LED red:").into(),
            if state.led_red() {
                Span::styled(
                    "ON",
                    Style::default()
                        .add_modifier(Modifier::ITALIC)
                        .fg(Color::Red),
                )
            } else {
                Span::styled(
                    "OFF",
                    Style::default()
                        .add_modifier(Modifier::ITALIC)
                        .fg(Color::DarkGray),
                )
            }
            .into(),
        ]),
    ];

    let table = Table::new(rows)
        .block(Block::default().title("State").borders(Borders::ALL))
        .widths(&[
            Constraint::Min(2),
            Constraint::Min(21),
            Constraint::Percentage(100),
        ])
        .column_spacing(2);

    frame.render_widget(table, area);
}

fn make_table_selection(
    row_value: BitFlags<GraphSelection>,
    current_selection: &BitFlags<GraphSelection>,
) -> Cell<'static> {
    let style = if current_selection.intersects(row_value) {
        Style::new().bold().underlined().fg(Color::White)
    } else {
        Style::new().fg(Color::DarkGray)
    };

    if let Some(row_value) = row_value.exactly_one() {
        let shortcut_number = (row_value as u8).trailing_zeros() + 1;
        Span::styled(format!("{shortcut_number}."), style)
    } else {
        Span::raw("")
    }
    .into()
}

fn render_keybindings(frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>, area: Rect) {
    let list = Paragraph::new(Line::from(vec![
        Span::styled("Q", Style::new().bold().underlined()),
        Span::raw("uit | "),
        Span::styled("E", Style::new().bold().underlined()),
        Span::raw("nable | "),
        Span::raw("<num> (to enable graphs) | "),
    ]))
    .block(
        Block::default()
            .title("Keybindings")
            .borders(Borders::ALL)
            .padding(Padding::horizontal(1)),
    );

    frame.render_widget(list, area);
}

fn render_error(
    frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>,
    area: Rect,
    state: &InductionHeaterState,
) {
    frame.render_widget(
        Paragraph::new(Span::styled(
            state.error().as_ref().unwrap().to_string(),
            Style::new().fg(Color::Red),
        )),
        area,
    );
}

fn render_graph(
    frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>,
    area: Rect,
    state: &InductionHeaterState,
    current_selection: &BitFlags<GraphSelection>,
) {
    let selection_count: usize = current_selection.into_iter().count();

    let graph_areas = Layout::default()
        .direction(Direction::Vertical)
        .constraints(
            current_selection
                .into_iter()
                .map(|_| Constraint::Ratio(1, selection_count as u32))
                .collect::<Vec<Constraint>>(),
        )
        .split(area);

    for (selection, area) in current_selection.into_iter().zip(graph_areas.iter()) {
        let data = state.get_data(selection);
        let dataset = Dataset::default()
            .marker(Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(Color::Yellow))
            .data(data);

        let data_horizontal_min = data
            .iter()
            .copied()
            .map(|(hor, _)| hor)
            .min_by(f64::total_cmp)
            .unwrap_or_default()
            .floor();
        let data_horizontal_max = data
            .iter()
            .copied()
            .map(|(hor, _)| hor)
            .max_by(f64::total_cmp)
            .unwrap_or_default()
            .ceil();

        let data_vertical_min = data
            .iter()
            .copied()
            .map(|(_, vert)| vert)
            .min_by(f64::total_cmp)
            .unwrap_or_default()
            .floor();
        let data_vertical_max = data
            .iter()
            .copied()
            .map(|(_, vert)| vert)
            .max_by(f64::total_cmp)
            .unwrap_or_default()
            .ceil();

        let locale = SystemLocale::default().unwrap();

        frame.render_widget(
            Chart::new(vec![dataset])
                .block(
                    Block::new()
                        .title(format!("{selection:?}"))
                        .borders(Borders::all()),
                )
                .y_axis(
                    Axis::default()
                        .bounds([data_vertical_min, data_vertical_max])
                        .labels(vec![
                            Span::raw(format!(
                                "{:^10}",
                                (data_vertical_min as i64).to_formatted_string(&locale)
                            )),
                            Span::raw(format!(
                                "{:^10}",
                                (((data_vertical_min + data_vertical_max) / 2.0).round() as i64)
                                    .to_formatted_string(&locale)
                            )),
                            Span::raw(format!(
                                "{:^10}",
                                (data_vertical_max as i64).to_formatted_string(&locale)
                            )),
                        ]),
                )
                .x_axis(
                    Axis::default()
                        .bounds([data_horizontal_min, data_horizontal_max])
                        .labels(vec![
                            Span::raw(format!(
                                "{}",
                                (data_horizontal_min as i64).to_formatted_string(&locale)
                            )),
                            Span::raw(format!(
                                "{}",
                                (((data_horizontal_min + data_horizontal_max) / 2.0).round()
                                    as i64)
                                    .to_formatted_string(&locale)
                            )),
                            Span::raw(format!(
                                "{}",
                                (data_horizontal_max as i64).to_formatted_string(&locale)
                            )),
                        ]),
                ),
            *area,
        );
    }
}

#[bitflags]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum GraphSelection {
    CoilDriveFrequency,
    CoilVoltageMax,
    FanRpm,
}
