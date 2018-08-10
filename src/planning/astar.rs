use super::world::*;

#[derive(Debug, Clone)]
pub enum Heuristic {
    Manhattan,
    Euclidean,
}

#[derive(Debug, Clone)]
pub struct AStarCfg {
    pub neighbors: Neighbors,
    pub heuristic: Option<Heuristic>,
    pub goal: Option<Id>,
    pub start: Option<Id>,
}

#[derive(Clone)]
pub struct AStar {
    config: AStarCfg,
    current: Option<Id>,
    frontier: Vec<(Id, f32)>, // cell Id, cost
    world: World,
    prev_step: usize,
}

impl AStarCfg {

    pub fn new() -> AStarCfg {
        AStarCfg {
            neighbors: Neighbors::Cardinal,
            heuristic: None,
            goal: None,
            start: None,
        }
    }

    pub fn with_goal(self, id: Id) -> AStarCfg {
        AStarCfg {
            neighbors: self.neighbors,
            heuristic: self.heuristic,
            goal: Some(id),
            start: self.start,
        }
    }

    pub fn with_start(self, id: Id) -> AStarCfg {
        AStarCfg {
            neighbors: self.neighbors,
            heuristic: self.heuristic,
            goal: self.goal,
            start: Some(id),
        }
    }

    pub fn with_hueristic(self, h: Option<Heuristic>) -> AStarCfg {
        AStarCfg {
            neighbors: self.neighbors,
            heuristic: h,
            goal: self.goal,
            start: self.start,
        }
    }

    pub fn with_neighbors(self, n: Neighbors) -> AStarCfg {
        AStarCfg {
            neighbors: n,
            heuristic: self.heuristic,
            goal: self.goal,
            start: self.start,
        }
    }

    pub fn valid_for(&self, world: &World) -> Result<(),String> {

        if let Some(goal) = self.goal {
            if None == world.coords_for(goal) {
                return Err("Invalid goal".to_string());
            }
        } else {
            return Err("Must specify goal point".to_string());
        }

        if let Some(start) = self.start {
            if None == world.coords_for(start) {
                return Err("Invalid start".to_string());
            }
        } else {
            return Err("Must specify start point".to_string());
        }

        Ok(())
    }

}

fn calc_euclidean_dist(a: (usize, usize), b: (usize, usize)) -> f32 {
    let (ax, ay) = a;
    let (bx, by) = b;
    (((ax.max(bx) - ax.min(bx)) as f32).powi(2) + 
     ((ay.max(by) - ay.min(by)) as f32).powi(2)).sqrt()
}

fn calc_manhattan_dist(a: (usize, usize), b: (usize, usize)) -> usize {
    let (ax, ay) = a;
    let (bx, by) = b;
    (ax.max(bx) - ax.min(bx)) + (ay.max(by) - ay.min(by))
}

impl AStar {

    pub fn from_cfg(cfg: AStarCfg, world: World) -> Result<AStar, String> {

        cfg.valid_for(&world)?;

        Ok(AStar {
            config: cfg,
            current: None,
            frontier: Vec::new(),
            world: world,
            prev_step: 0,
        })
    }

    pub fn step(&mut self) -> Option<usize> {

        let goal = self.config.goal.unwrap();
        // get the next cell
        let next: Id = match self.current {
            // first time step is called, use goal
            None => {
                *self.world.cell_mut(goal).unwrap() =
                    Cell::Visited {
                        g: 0.0,
                        h: 0.0,
                        k: 0.0,
                        parent: goal
                    };
                goal
            }
            Some(_id) => {
                // pull the best from the frontier
                if let Some((id,_)) = self.frontier.pop() { id }
                // empty frontier? search was already completed
                else { return None }
            }
        };

        // check if done
        if next == self.config.start.unwrap() { return None };

        // get x, y coordinates for current cell
        let my_coord = self.world.coords_for(next).unwrap();
        // get x, y coordinates for current goal
        let goal_coord = self.world.coords_for(self.config.start.unwrap()).unwrap();

        let my_cost = match *self.world.cell(next).unwrap() {
            Cell::Visited { g, h:_, k:_, parent:_ } => g,
            _ => 0.0f32
        };

        // determine neighbors, calc costs, add to frontier
        let neighbors =
            self.world.iter_neighbor_ids(next, self.config.neighbors).unwrap();

        for (x,y) in neighbors {
            // a way to signal that we need to add to frontier after updates
            let mut add_to_frontier: Option<f32> = None;
            if let Some(cell) = self.world.cell_at_mut(x,y) {
                // skip obstacles
                if let Cell::Obstacle = cell { continue };
                // determine cost to go
                let new_cost = match self.config.neighbors {
                    Neighbors::CardinalAndDiagonal =>
                        calc_euclidean_dist((x,y), my_coord),
                    Neighbors::Cardinal => 1.0,
                } + my_cost;
                // determine heuristic
                let new_heur = match self.config.heuristic {
                    Some(Heuristic::Euclidean) =>
                        calc_euclidean_dist((x,y), goal_coord),
                    Some(Heuristic::Manhattan) =>
                        calc_manhattan_dist((x,y), goal_coord) as f32,
                    None => 0.0,
                };
                // build Cell data - optionally used below
                let new_cell = Cell::Visited {
                    g: new_cost,
                    h: new_heur,
                    k: 0.0, // TODO is this ok?
                    parent: next,
                };
                // update cell if unvisited or better than frontier options
                match *cell {
                    // if visited then we already have it in the frontier, 
                    // just update
                    Cell::Visited { g, h:_, k:_, parent:_ } =>
                        if g > new_cost {
                            add_to_frontier = Some(new_heur + new_cost);
                            *cell = new_cell;
                        },
                    // if open then its unvisited and needs to be added to the
                    // frontier list and updated
                    Cell::Open => {
                        add_to_frontier = Some(new_heur + new_cost);
                        *cell = new_cell;
                    },
                    // this match arm should never hit
                    _ => { },
                };
            }
            // replace or add to frontier
            if let Some(cost) = add_to_frontier {
                let id = self.world.id_at(x,y).unwrap();
                let location = self.frontier.iter().position(
                    |&(i,_)| i == id
                );
                match location {
                    Some(idx) => self.frontier[idx] = (id, cost),
                    None      => self.frontier.push((id, cost))
                };
            }
        }

        self.frontier.sort_by(|a, b| {
            let (_, cost_a) = a;
            let (_, cost_b) = b;
            cost_b.partial_cmp(cost_a).unwrap()
        });
        self.prev_step += 1;
        self.current = Some(next);
        Some(self.prev_step)
    }

    pub fn world_view(&self) -> &World {
        &self.world
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_dist_funcs() {
        assert_eq!(calc_manhattan_dist((0,0),(1,1)), 2);
        assert_eq!(calc_manhattan_dist((1,1),(0,0)), 2);
        assert_eq!(calc_manhattan_dist((0,1),(1,0)), 2);
        assert!(calc_euclidean_dist(
            (0,0),(1,1))-(2.0f32).sqrt().abs() < 1e-6
        );
        assert!(calc_euclidean_dist(
            (1,1),(0,0))-(2.0f32).sqrt().abs() < 1e-6
        );
    }

    #[test]
    fn test_simple_pathing() {

        let w = World::new( 4, 8, vec![
            Cell::Open, Cell::Obstacle, Cell::Open, Cell::Open,
            Cell::Open, Cell::Obstacle, Cell::Open, Cell::Open,
            Cell::Open, Cell::Open,     Cell::Open, Cell::Open,
            Cell::Open, Cell::Open,     Cell::Open, Cell::Open,
            Cell::Open, Cell::Open,     Cell::Open, Cell::Open,
            Cell::Open, Cell::Open,     Cell::Open, Cell::Open,
            Cell::Open, Cell::Open,     Cell::Open, Cell::Open,
            Cell::Open, Cell::Open,     Cell::Open, Cell::Open,
        ]).unwrap();

        let cfg = AStarCfg::new()
                    .with_goal(w.id_at(0,0).unwrap())
                    .with_start(w.id_at(3,0).unwrap())
                    .with_hueristic(Some(Heuristic::Euclidean))
                    .with_neighbors(Neighbors::Cardinal);

        println!("{:#?}",cfg);

        let mut astar = AStar::from_cfg(cfg, w).unwrap();

        let mut last_step = 0;
        loop {
            println!("Step {}",last_step);
            match astar.step() {
                Some(step) => last_step = step,
                None => break
            };
        }

        let wv = astar.world_view();
        for i in 0..wv.height() {
            for j in 0..wv.width() {
                print!("{}\t\t",wv.cell_at(j,i).unwrap());
            }
            println!("");
        }
    }
}
