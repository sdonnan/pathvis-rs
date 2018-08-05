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

pub use world::World;
pub use world_controller::WorldController;
pub use world_view::{WorldView, WorldViewSettings};

mod world;
mod world_controller;
mod world_view;

fn main() {
    let opengl = OpenGL::V3_2;
    let settings = WindowSettings::new("Path Visualizer", [512; 2])
        .opengl(opengl)
        .exit_on_esc(true);
    let mut window: GlutinWindow = settings.build()
        .expect("Could not create window");
    let mut events = Events::new(EventSettings::new().lazy(true));
    let mut gl = GlGraphics::new(opengl);

    let world = World::new();
    let mut world_controller = WorldController::new(world);
    let world_view_settings = WorldViewSettings::new();
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
