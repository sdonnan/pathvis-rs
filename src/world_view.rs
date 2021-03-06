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
    pub position: [f64; 2],
    pub size: f64,
    pub font_size: u32,
    pub background_color: Color,
    pub border_color: Color,
    pub board_edge_color: Color,
    pub cell_edge_color: Color,
    pub board_edge_radius: f64,
    pub cell_edge_radius: f64,
    pub text_color: Color,
    pub goal_color: Color,
    pub start_color: Color,
    pub blocked_cell_color: Color,
    pub open_cell_color: Color,
    pub path_line_color: Color,
    pub path_line_radius: f64,
    pub visited_cell_color: Color,
}

impl WorldViewSettings {
    /// Creates new world view settings.
    pub fn new() -> WorldViewSettings {
        WorldViewSettings {
            position: [10.0; 2],
            size: 600.0,
            font_size: 18,
            background_color: [0.8, 0.8, 1.0, 1.0],
            border_color: [0.0, 0.0, 0.2, 1.0],
            board_edge_color: [0.0, 0.0, 0.2, 1.0],
            cell_edge_color: [0.0, 0.0, 0.2, 1.0],
            board_edge_radius: 3.0,
            cell_edge_radius: 1.0,
            text_color: [0.0, 0.0, 0.1, 1.0],
            goal_color: [0.6, 0.8, 0.6, 1.0],
            start_color: [0.6, 0.6, 1.0, 1.0],
            blocked_cell_color: [0.3, 0.3, 0.3, 1.0],
            open_cell_color: [0.6, 0.6, 0.8, 1.0],
            path_line_color: [1.0, 0.5, 1.0, 1.0],
            path_line_radius: 5.0,
            visited_cell_color: [1.0, 0.9, 1.0, 1.0],
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
        use graphics::{Text, Rectangle, Transformed};
        let (x_, y_) = pos;
        let (x, y) = (self.settings.position[0] + x_, self.settings.position[1] + y_);
        let (sx, sy) = size;
        let rad = self.settings.board_edge_radius;
        let label_rect = [x + rad, y + rad, sx - 2.0 * rad, sy - 2.0 * rad];
        Rectangle::new_round(self.settings.background_color, self.settings.cell_edge_radius)
            .draw(label_rect, &c.draw_state, c.transform, g);
        Rectangle::new_round_border(self.settings.cell_edge_color,
                                 self.settings.cell_edge_radius,
                                 self.settings.cell_edge_radius)
            .draw(label_rect, &c.draw_state, c.transform, g);
        let text_image = Text::new(self.settings.font_size);
        text_image.draw(text,
                        glyphs,
                        &c.draw_state,
                        c.transform.trans(x + 10.0, y + 0.5 * (sy + self.settings.font_size as f64)),
                        g);
    }

    fn write_anywhere <G: Graphics, C>(
      &self,
      pos: (f64, f64),
      text: &str,
      glyphs: &mut C,
      c: &Context,
      g: &mut G,
    ) 
      where C: CharacterCache<Texture = G::Texture>
    {
        use graphics::{Text, Transformed};
        let (x_, y_) = pos;
        let (x, y) = (self.settings.position[0] + x_, self.settings.position[1] + y_);
        let text_image = Text::new(self.settings.font_size);
        text_image.draw(text,
                        glyphs,
                        &c.draw_state,
                        c.transform.trans(x, y),
                        g);
    }

    fn write_cell<G: Graphics, C>(
      &self,
      cell_size: f64,
      cell: (usize, usize),
      pos: (f64, f64),
      text: &str,
      glyphs: &mut C,
      c: &Context,
      g: &mut G,
    ) 
      where C: CharacterCache<Texture = G::Texture>
    {
        use graphics::{Text, Transformed};
        let (i,j) = cell;
        let pos = [i as f64 * cell_size + self.settings.board_edge_radius + pos.0,
                   j as f64 * cell_size + self.settings.font_size as f64 + pos.1];
        let text_image = Text::new(self.settings.font_size);
        text_image.draw(text,
                        glyphs,
                        &c.draw_state,
                        c.transform.trans(pos[0] + self.settings.position[0],
                                          pos[1] + self.settings.position[1]),
                        g);
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
        use graphics::{Line, Rectangle, Text, Transformed};

        let ref settings = self.settings;
        let board_rect = [
            settings.position[0], settings.position[1],
            settings.size, settings.size,
        ];

        // Draw board background.
        Rectangle::new(settings.background_color)
            .draw(board_rect, &c.draw_state, c.transform, g);

        let cell_size = settings.size / controller.world().width() as f64;

        // Number cells
        for j in 0..controller.world().height() {
            let pos = [ 0.0 - self.settings.font_size as f64, j as f64 * cell_size + (self.settings.font_size*2) as f64];
            let text_image = Text::new(self.settings.font_size);
            text_image.draw(&format!("{}",j),
                            glyphs,
                            &c.draw_state,
                            c.transform.trans(pos[0] + self.settings.position[0],
                                              pos[1] + self.settings.position[1]),
                            g);
        }
        for j in 0..controller.world().width() {
            let pos = [ j as f64 * cell_size + (self.settings.font_size*2) as f64, 0.0 - self.settings.board_edge_radius * 2.0];
            let text_image = Text::new(self.settings.font_size);
            text_image.draw(&format!("{}",j),
                            glyphs,
                            &c.draw_state,
                            c.transform.trans(pos[0] + self.settings.position[0],
                                              pos[1] + self.settings.position[1]),
                            g);
        }

        // Draw cells.
        for j in 0..controller.world().height() {
            for i in 0..controller.world().width() {
                let cell = controller.world().cell_at(i, j).unwrap();
                let cell_id = controller.world().id_at(i, j).unwrap();
                let pos = [i as f64 * cell_size, j as f64 * cell_size];

                // draw background
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

                // Mark start and goal
                if let Some(start) = controller.state.start() {
                    if start == cell_id {
                        Rectangle::new(settings.start_color).draw(cell_rect, &c.draw_state, c.transform, g);
                        self.write_cell(cell_size, (i,j), (cell_size - settings.font_size as f64,0.0),
                                        "S", glyphs, c, g);                     
                    }
                }
                if let Some(goal) = controller.state.goal() {
                    if goal == cell_id {
                        Rectangle::new(settings.goal_color).draw(cell_rect, &c.draw_state, c.transform, g);
                        self.write_cell(cell_size, (i,j), (cell_size - settings.font_size as f64,0.0),
                                        "G", glyphs, c, g);                     
                    }
                }

            }
        }

        // Draw selected cell border as bold
        if let AppState::Active(astar) = &controller.state {
            if let Some(cell) = astar.current() {
                let (ind_x, ind_y) = astar.world_view().coords_for(cell).unwrap();
                let pos = [ind_x as f64 * cell_size, ind_y as f64 * cell_size];
                let cell_rect = [
                    settings.position[0] + pos[0], settings.position[1] + pos[1],
                    cell_size, cell_size
                ];
                Rectangle::new_border(self.settings.board_edge_color, self.settings.board_edge_radius)
                    .draw(cell_rect, &c.draw_state, c.transform, g);
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

        // Draw path
        let path_line = Line::new_round(settings.path_line_color, settings.path_line_radius);
        if let AppState::Active(astar) = &controller.state {
            if let Some(path) = astar.path() {
                let mut ids = path.iter();
                let mut prev_coord = astar.world_view().coords_for(*ids.next().unwrap()).unwrap();
                for id in ids {
                    let (x1,y1) = prev_coord;
                    let (x2,y2) = astar.world_view().coords_for(*id).unwrap();
                    path_line.draw([x1 as f64 * cell_size + cell_size * 0.5 + settings.position[0],
                                    y1 as f64 * cell_size + cell_size * 0.5 + settings.position[1],
                                    x2 as f64 * cell_size + cell_size * 0.5 + settings.position[0],
                                    y2 as f64 * cell_size + cell_size * 0.5 + settings.position[1]],
                                    &c.draw_state, c.transform, g);
                    prev_coord = (x2, y2);
                }
            }
        }

        // Draw text in visited cells over top of everything else
        for j in 0..controller.world().height() {
            for i in 0..controller.world().width() {
                let cell = controller.world().cell_at(i, j).unwrap();

                // Fill visited
                if let Cell::Visited{g: goalcost, h: heurcost, k: _, parent} = cell {
                    self.write_cell(cell_size, (i,j), (0.0, (settings.font_size * 0) as f64), 
                                    &format!("g: {:0.1}", goalcost), glyphs, c, g);                     
                    self.write_cell(cell_size, (i,j), (0.0, (settings.font_size * 1) as f64), 
                                    &format!("h: {:0.1}", heurcost), glyphs, c, g);                     
                    self.write_cell(cell_size, (i,j), (0.0, (settings.font_size * 2) as f64), 
                                    &format!("p: {:?}", controller.world().coords_for(*parent).unwrap()), glyphs, c, g);                     
                };

            }
        }


        // Draw board edge.
        Rectangle::new_border(settings.board_edge_color, settings.board_edge_radius)
            .draw(board_rect, &c.draw_state, c.transform, g);

        // Draw controlls (another column past the board of 1x2 cells)
        let mut labels: Vec<String> = Vec::new();
        let mut stats: Vec<String> = Vec::new();
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
                let mut message = "Left click to toggle obstacles, ".to_string();
                if let Ok(_) = cfg.valid_for(&world) {
                    labels.push("Start".to_string());
                    message = message + "Right click to reset Start and Goal.";
                } else {
                    message = message + "Right click to set Start and Goal.";
                }
                self.write_anywhere((self.settings.position[0], 
                                     self.settings.position[1] + self.settings.size), 
                                     &message,
                                     glyphs, c, g);       
            },
            AppState::Active(astar) => { 
                let cell_string : String  = if let Some(id) = astar.current() {
                    let (x,y) = astar.world_view().coords_for(id).unwrap();
                    let cost = if let Cell::Visited{g: goalcost, h: heurcost, k: _, parent:_} = 
                                      astar.world_view().cell(id).unwrap() {
                            goalcost + heurcost
                        } else { 0.0 };
                    format!("Current Cell: ({},{}) f: {:0.1}",x,y,cost)
                } else { String::new() };

                labels.push( "Next".to_string() );
                labels.push( "Reset".to_string() );
                stats.push( cell_string );
                stats.push(  String::new() );
                stats.push( "Frontier:".to_string() );
                let fview = astar.frontier_view();
                for idx in 1..(fview.len()+1) {
                    let (id, cost) = fview.get(fview.len() - idx).unwrap();
                    let (x,y) = astar.world_view().coords_for(*id).unwrap();
                    stats.push(format!("  {}. ({},{}) f: {:0.1}",idx,x,y,cost))
                }
                let mut message = "Click next to advance planning. ".to_string();
                message += match astar.config_view().heuristic { 
                        None => "Using no heuristic",
                        Some(Heuristic::Euclidean) => "Using Euclidean distance as hueristic",
                        Some(Heuristic::Manhattan) => "Using Manhattan distance as hueristic",
                };
                message += match astar.config_view().neighbors { 
                        Neighbors::CardinalAndDiagonal => " and allowing diagonal moves.",
                        Neighbors::Cardinal => " and not allowing diagonal moves.",
                };
                self.write_anywhere((self.settings.position[0], 
                                     self.settings.position[1] + self.settings.size), 
                                     &message,
                                     glyphs, c, g);       
            },
        }
        let mut index = 0;
        for label in labels {
            self.draw_label((settings.size + 10.0, index as f64 * cell_size), (cell_size * 3.0, cell_size), &label, glyphs, c, g);       
            index += 1;
        }
        let offset = index as f64 * cell_size;
        for stat in stats {
            self.write_anywhere((settings.size + 10.0, (index * self.settings.font_size) as f64 + offset), &stat, glyphs, c, g);       
            index += 1;
        }
    }
}
