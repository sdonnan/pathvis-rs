//! World view.

use graphics::types::Color;
use graphics::{Context, Graphics};
use graphics::character::CharacterCache;

use WorldController;
use AppState;
use planning::world::*;
use planning::astar::*;

/// Stores world view settings.
pub struct WorldViewSettings {
    /// Position from left-top corner.
    pub position: [f64; 2],
    /// Size of world along horizontal and vertical edge.
    pub size: f64,
    pub font_size: u32,
    /// Background color.
    pub background_color: Color,
    /// Border color.
    pub border_color: Color,
    /// Edge color around the whole board.
    pub board_edge_color: Color,
    /// Edge color between the 3x3 sections.
    pub section_edge_color: Color,
    /// Edge color between cells.
    pub cell_edge_color: Color,
    /// Edge radius around the whole board.
    pub board_edge_radius: f64,
    /// Edge radius between the 3x3 sections.
    pub section_edge_radius: f64,
    /// Edge radius between cells.
    pub cell_edge_radius: f64,
    /// Color of selected cell
    pub selected_cell_background_color: Color,
    /// Text color.
    pub text_color: Color,
    pub blocked_cell_color: Color,
    pub open_cell_color: Color,
    pub visited_cell_color: Color,
}

impl WorldViewSettings {
    /// Creates new world view settings.
    pub fn new() -> WorldViewSettings {
        WorldViewSettings {
            position: [10.0; 2],
            size: 400.0,
            font_size: 18,
            background_color: [0.8, 0.8, 1.0, 1.0],
            border_color: [0.0, 0.0, 0.2, 1.0],
            board_edge_color: [0.0, 0.0, 0.2, 1.0],
            section_edge_color: [0.0, 0.0, 0.2, 1.0],
            cell_edge_color: [0.0, 0.0, 0.2, 1.0],
            board_edge_radius: 3.0,
            section_edge_radius: 2.0,
            cell_edge_radius: 1.0,
            selected_cell_background_color: [0.9, 0.9, 1.0, 1.0],
            text_color: [0.0, 0.0, 0.1, 1.0],
            blocked_cell_color: [0.3, 0.3, 0.3, 1.0],
            open_cell_color: [0.8, 0.8, 1.0, 1.0],
            visited_cell_color: [0.9, 0.9, 1.0, 1.0],
        }
    }
}

/// Stores visual information about a world.
pub struct WorldView {
    /// Stores world view settings.
    pub settings: WorldViewSettings,
}

impl WorldView {
    /// Creates a new world view.
    pub fn new(settings: WorldViewSettings) -> WorldView {
        WorldView {
            settings: settings,
        }
    }

    fn draw_label<G: Graphics, C>(
      &self,
      pos: (f64, f64),
      size: (f64, f64),
      text: &str,
      glyphs: &mut C,
      c: &Context,
      g: &mut G,
    ) 
      where C: CharacterCache<Texture = G::Texture>
    {
        use graphics::{Text, Image, Line, Rectangle, Transformed};
        let (x_, y_) = pos;
        let (x, y) = (self.settings.position[0] + x_, self.settings.position[1] + y_);
        let (sx, sy) = size;
        let text_image = Text::new(self.settings.font_size);
        text_image.draw(text,
                        glyphs,
                        &c.draw_state,
                        c.transform.trans(x + 10.0, y + 0.5 * (sy + self.settings.font_size as f64)),
                        g);
        let label_rect = [x, y, sx, sy];
        Rectangle::new_border(self.settings.board_edge_color, self.settings.board_edge_radius)
            .draw(label_rect, &c.draw_state, c.transform, g);
    }


    /// Draw world.
    pub fn draw<G: Graphics, C>(
      &self,
      controller: &WorldController,
      glyphs: &mut C,
      c: &Context,
      g: &mut G
    )
      where C: CharacterCache<Texture = G::Texture>
    {
        use graphics::{Text, Image, Line, Rectangle, Transformed};

        let ref settings = self.settings;
        let board_rect = [
            settings.position[0], settings.position[1],
            settings.size, settings.size,
        ];

        // Draw board background.
        Rectangle::new(settings.background_color)
            .draw(board_rect, &c.draw_state, c.transform, g);

        let cell_size = settings.size / controller.world().width() as f64;
        
        // Draw selected cell background.
        if let Some(ind) = controller.selected_cell {
            let (ind_x, ind_y) = ind;
            let pos = [ind_x as f64 * cell_size, ind_y as f64 * cell_size];
            let cell_rect = [
                settings.position[0] + pos[0], settings.position[1] + pos[1],
                cell_size, cell_size
            ];
            Rectangle::new(settings.selected_cell_background_color)
                .draw(cell_rect, &c.draw_state, c.transform, g);
        }

        // Draw cells.
        for j in 0..controller.world().height() {
            for i in 0..controller.world().width() {
                let cell = controller.world().cell_at(i, j).unwrap();
                let pos = [i as f64 * cell_size, j as f64 * cell_size];
                let cell_rect = [
                    settings.position[0] + pos[0], settings.position[1] + pos[1],
                    cell_size, cell_size
                ];
                let color = match cell {
                    Cell::Obstacle => settings.blocked_cell_color,
                    Cell::Open => settings.open_cell_color,
                    _ => settings.visited_cell_color,
                };
                Rectangle::new(color).draw(cell_rect, &c.draw_state, c.transform, g);
            }
        }

        // Draw cell borders.
        let cell_edge = Line::new(settings.cell_edge_color, settings.cell_edge_radius);
        for i in 0..controller.world().width() {

            let x = settings.position[0] + i as f64 / controller.world().width() as f64 * settings.size;
            let y = settings.position[1] + i as f64 / controller.world().height() as f64 * settings.size;
            let x2 = settings.position[0] + settings.size;
            let y2 = settings.position[1] + settings.size;

            let vline = [x, settings.position[1], x, y2];
            cell_edge.draw(vline, &c.draw_state, c.transform, g);

            let hline = [settings.position[0], y, x2, y];
            cell_edge.draw(hline, &c.draw_state, c.transform, g);
        }

        // Draw board edge.
        Rectangle::new_border(settings.board_edge_color, settings.board_edge_radius)
            .draw(board_rect, &c.draw_state, c.transform, g);

        // Draw controlls (another column past the board of 1x2 cells)
        let mut labels: Vec<String> = Vec::new();
        match &controller.state {
            AppState::Config{cfg, world} => {
                labels.push(
                    match cfg.neighbors { 
                        Neighbors::CardinalAndDiagonal => "Diagonal: Yes".to_string(),
                        Neighbors::Cardinal => "Diagonal: No".to_string(),
                    }
                );
                labels.push(
                    match cfg.heuristic { 
                        Some(Heuristic::Manhattan) => "Heuristic: Manhattan".to_string(),
                        Some(Heuristic::Euclidean) => "Heuristic: Euclidean".to_string(),
                        None => "Heuristic: None".to_string(),
                    }
                );
                if let Ok(_) = cfg.valid_for(&world) {
                    labels.push("Start".to_string());
                }
            },
            AppState::Active(_) => { 
                labels.push( "Next".to_string() );
                labels.push( "Prev".to_string() );
                labels.push( "Reset".to_string() );
                labels.push( "Current Cell:".to_string() );
                labels.push( "Frontier:".to_string() );
            },
        }
        let mut index = 0;
        for label in labels {
            self.draw_label((settings.size + 10.0, index as f64 * cell_size), (cell_size * 3.0, cell_size), &label, glyphs, c, g);       
            index += 1;
        }
    }
}
