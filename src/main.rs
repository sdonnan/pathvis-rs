extern crate piston;
extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;

use opengl_graphics::{
    Filter,
    GlGraphics,
    GlyphCache,
    OpenGL,
    TextureSettings,
};
use piston::window::WindowSettings;
use piston::event_loop::{
    EventLoop,
    Events,
    EventSettings,
};
use piston::input::RenderEvent;
use glutin_window::GlutinWindow;

pub use planning::world::*;
pub use planning::astar::*;
pub use world_controller::{WorldController, AppState};
pub use world_view::{WorldView, WorldViewSettings};

mod planning;
mod world_controller;
mod world_view;

fn main() {
    let opengl = OpenGL::V3_2;
    let world_side_len: u32 = 10; // number of cells in x and y directions
    let pts_per_cell: u32 = 64; // set the size of the cells on screen
    let settings = WindowSettings::new("Path Visualizer", [pts_per_cell * (world_side_len + 5), 
                                                           pts_per_cell * (world_side_len + 2)])
        .opengl(opengl)
        .exit_on_esc(true);
    let mut window: GlutinWindow = settings.build()
        .expect("Could not create window");
    let mut events = Events::new(EventSettings::new().lazy(true));
    let mut gl = GlGraphics::new(opengl);

    let mut world_controller = WorldController::new(world_side_len as usize);
    let mut world_view_settings = WorldViewSettings::new();
    world_view_settings.size = (pts_per_cell * world_side_len) as f64;
    world_view_settings.font_size = (pts_per_cell as f64 / 4.0) as u32; // imperically determined ratio
    world_view_settings.position = [(pts_per_cell/2) as f64; 2]; 
    let world_view = WorldView::new(world_view_settings);
    let texture_settings = TextureSettings::new().filter(Filter::Nearest);
    let ref mut glyphs = GlyphCache::new("assets/FiraSans-Regular.ttf", (), texture_settings)
      .expect("Could not load font");

    while let Some(e) = events.next(&mut window) {
        world_controller.event(world_view.settings.position,
                               world_view.settings.size,
                               &e);
        if let Some(args) = e.render_args() {
            gl.draw(args.viewport(), |c, g| {
                use graphics::{clear};

                clear([1.0; 4], g);
                world_view.draw(&world_controller, glyphs, &c, g);
            });
        }
    }
}
