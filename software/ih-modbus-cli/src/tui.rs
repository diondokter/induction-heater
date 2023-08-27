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
use ratatui::{
    prelude::*,
    text::Spans,
    widgets::{List, ListItem, *},
};

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
    loop {
        terminal.draw(|frame| render_app(frame, &state))?;
        if event::poll(Duration::from_millis(16)).context("event poll failed")? {
            match event::read().context("event read failed")? {
                Event::Key(key) => {
                    if key.code == KeyCode::Char('q') {
                        break;
                    }
                    if key.code == KeyCode::Char('e') && key.kind == KeyEventKind::Press {
                        let mut state_guard = state.lock().unwrap();
                        let flipped_target = !state_guard.target_enabled();
                        state_guard.set_target_enabled(flipped_target);
                    }
                }
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
) {
    let state = state.lock().unwrap().clone();

    let top_bottom = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(100), Constraint::Min(3)].as_ref())
        .split(frame.size());

    let main_window = top_bottom[0];
    let keybindings_windows = top_bottom[1];

    // Create two chunks with equal horizontal screen space
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)].as_ref())
        .split(main_window);

    render_table_state(frame, chunks[0], &state);
    render_keybindings(frame, keybindings_windows);
}

fn render_table_state(
    frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>,
    area: Rect,
    state: &InductionHeaterState,
) {
    let rows = vec![
        Row::new(vec![
            Span::raw("Enabled: "),
            if state.led_green() {
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
            },
        ]),
        Row::new(vec![
            Span::raw("Coil drive frequency: "),
            Span::raw(format!("{}", state.coil_drive_frequency())),
        ]),
        Row::new(vec![
            Span::raw("Coil voltage max: "),
            Span::raw(format!("{}", state.coil_voltage_max())),
        ]),
        Row::new(vec![
            Span::raw("Fan RPM: "),
            Span::raw(format!("{}", state.fan_rpm())),
        ]),
        Row::new(vec![
            Span::raw("LED green: "),
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
            },
        ]),
        Row::new(vec![
            Span::raw("LED red: "),
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
            },
        ]),
    ];

    let table = Table::new(rows)
        .block(Block::default().title("State").borders(Borders::ALL))
        .widths(&[Constraint::Ratio(1, 3), Constraint::Ratio(2, 3)])
        .column_spacing(5);

    frame.render_widget(table, area);
}

fn render_keybindings(frame: &mut ratatui::Frame<CrosstermBackend<Stdout>>, area: Rect) {
    let list = Paragraph::new(Line::from(vec![
        Span::styled("Q", Style::new().bold().underlined()),
        Span::raw("uit | "),
        Span::styled("E", Style::new().bold().underlined()),
        Span::raw("nable | "),
    ]))
    .block(Block::default().title("Keybindings").borders(Borders::ALL).padding(Padding::horizontal(1)));

    frame.render_widget(list, area);
}
