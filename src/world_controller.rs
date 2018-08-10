//! World controller.

use piston::input::GenericEvent;

use planning::world::*;
use planning::astar::*;

pub enum AppState {
    Config {
        cfg: AStarCfg,
        world: World,
    },
    Active(AStar),
}

/// Handles events for Sudoku game.
pub struct WorldController {
    /// Determines current state
    pub state: AppState,
    /// Selected cell.
    pub selected_cell: Option<(usize, usize)>,
    /// Stores last mouse cursor position.
    pub cursor_pos: [f64; 2],
}

impl WorldController {
    /// Creates a new world controller.
    pub fn new() -> WorldController {
        WorldController {
            state: AppState::Config{ cfg: AStarCfg::new(), world: World::new(6,6,vec![Cell::Open; 36]).unwrap()},
            selected_cell: None,
            cursor_pos: [0.0, 1.0],
        }
    }

    pub fn world(&self) -> &World {
        match &self.state {
            AppState::Config { cfg: _, world } => &world,
            AppState::Active(astar) => astar.world_view(),
        }
    }

    /// Handles events.
    pub fn event<E: GenericEvent>(&mut self, pos: [f64; 2], size: f64, e: &E) {
        use piston::input::{Button, Key, MouseButton};

        // layout unit of measure
        let cell_size = size / self.world().width() as f64;
  
        if let Some(pos) = e.mouse_cursor_args() {
            self.cursor_pos = pos;
        }
        if let Some(Button::Mouse(MouseButton::Right)) = e.press_args() {
            // Find coordinates relative to upper left corner.
            let x = self.cursor_pos[0] - pos[0];
            let y = self.cursor_pos[1] - pos[1];
            // Check that coordinates are inside board boundaries.
            if x >= 0.0 && x <= size && y >= 0.0 && y <= size {
                // Compute the cell position.
                let cell_x = (x / size * self.world().width() as f64) as usize;
                let cell_y = (y / size * self.world().height() as f64) as usize;
                self.selected_cell = Some((cell_x, cell_y));
                match &mut self.state {
                    AppState::Config { cfg, world} => {
                        // Set goal then start
                        if cfg.start == None {
                            cfg.start = world.id_at(cell_x, cell_y);
                        } else if cfg.goal == None {
                            cfg.goal = world.id_at(cell_x, cell_y);
                        } else {
                            cfg.start = None;
                            cfg.goal = None;
                        }
                    }
                    AppState::Active(_) => {}
                };

            }
        }
        
        if let Some(Button::Mouse(MouseButton::Left)) = e.press_args() {
            // Find coordinates relative to upper left corner.
            let x = self.cursor_pos[0] - pos[0];
            let y = self.cursor_pos[1] - pos[1];
            // Check that coordinates are inside board boundaries.
            if x >= 0.0 && x <= size && y >= 0.0 && y <= size {
                // Compute the cell position.
                let cell_x = (x / size * self.world().width() as f64) as usize;
                let cell_y = (y / size * self.world().height() as f64) as usize;
                self.selected_cell = Some((cell_x, cell_y));
                match &mut self.state {
                    AppState::Config { cfg: _, world} => {
                        // Toggle obstacle if in config state
                        if let Some(cell) = world.cell_at_mut(cell_x, cell_y) {
                            *cell = match cell {
                                Cell::Obstacle => Cell::Open,
                                _              => Cell::Obstacle,
                            }
                        }
                    }
                    AppState::Active(_) => {}
                };

            }
            // Check that coordinates are inside controls boundaries.
            if x >= size && x <= size + cell_size * 3.0 && y >= 0.0 && y <= size {
                // Compute the cell position.
                let ctrl_index = (y / cell_size) as usize;
                let mut toggle_state = false;
                match &mut self.state {
                    AppState::Config{cfg, world} => {
                        match ctrl_index {
                            0 => match cfg.neighbors {
                                Neighbors::CardinalAndDiagonal => cfg.neighbors = Neighbors::Cardinal,
                                Neighbors::Cardinal => cfg.neighbors = Neighbors::CardinalAndDiagonal,
                            },
                            1 => match cfg.heuristic {
                                Some(Heuristic::Manhattan) => cfg.heuristic = None,
                                Some(Heuristic::Euclidean) => cfg.heuristic = Some(Heuristic::Manhattan),
                                None => cfg.heuristic = Some(Heuristic::Euclidean),
                            },
                            2 => {
                                if let Ok(_) = cfg.valid_for(&world) {
                                    toggle_state = true;
                                }
                            },
                            _ => {},
                        };
                    },
                    AppState::Active(astar) => {
                        match ctrl_index {
                            0 => {astar.step();},
                            1 => {}
                            2 => {
                                toggle_state = true;
                            },
                            _ => {},
                        };
                    }
                };
                if toggle_state {
                    let new_state = match &self.state {
                        AppState::Config{cfg, world} => AppState::Active(
                            AStar::from_cfg(cfg.clone(),
                            world.clone()).unwrap()
                        ),
                        AppState::Active(_) => AppState::Config{ 
                            cfg: AStarCfg::new(), 
                            world: World::new(6,6,vec![Cell::Open; 36]).unwrap()
                        },
                    };
                    self.state = new_state;
                }
            }
        }
        if let Some(Button::Keyboard(key)) = e.press_args() {
            if let Some(ind) = self.selected_cell {
                // Set cell value.
                match key {
//                    Key::D1 => self.world.set(ind, 1),
//                    Key::D2 => self.world.set(ind, 2),
//                    Key::D3 => self.world.set(ind, 3),
//                    Key::D4 => self.world.set(ind, 4),
//                    Key::D5 => self.world.set(ind, 5),
//                    Key::D6 => self.world.set(ind, 6),
//                    Key::D7 => self.world.set(ind, 7),
//                    Key::D8 => self.world.set(ind, 8),
//                    Key::D9 => self.world.set(ind, 9),
                    _ => {}
                }
            }
        }
    }
}

