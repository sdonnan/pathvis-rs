use std::fmt;

/// Represent the state of a cell in the world
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cell {
    Obstacle,
    Open,
    Visited {
        g: f32, // distance transform
        h: f32, // hueristic
        k: f32, // key value
        parent: Id
    },
}

impl fmt::Display for Cell {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Cell::Obstacle => write!(f, "~BLOCKED~"),
            Cell::Open     => write!(f, "~~OPEN~~~"),
            Cell::Visited { g, h, k: _, parent } => 
                write!(f, "{:0.1} {:0.1} {}", g, h, parent)
        }
    }
}

/// A collection of cells defining a 2D world
#[derive(Debug, Clone)]
pub struct World {
    width: usize,
    height: usize,
    cells: Vec<Cell>,
}

/// A way to describe neighbor strategies
#[derive(Clone, Copy, Debug)]
pub enum Neighbors {
    Cardinal,
    CardinalAndDiagonal,
}

/// Neighbor directions
#[derive(Clone, Copy, Debug)]
enum Neighbor {
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW,
}

pub struct NeighborIter {
    next: Option<Neighbor>,
    strat: Neighbors,
    x: usize,
    y: usize,
}

impl NeighborIter {
    fn new(x: usize, y: usize, strat: Neighbors) -> NeighborIter {
        NeighborIter {
            next: Some(Neighbor::E),
            strat: strat,
            x: x,
            y: y,
        }
    }
}

impl Iterator for NeighborIter {
    type Item = (usize, usize);

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(dir) = self.next {
            match dir {
                Neighbor::E => {
                    match self.strat {
                        Neighbors::CardinalAndDiagonal => 
                            self.next = Some(Neighbor::SE),
                        Neighbors::Cardinal => 
                            self.next = Some(Neighbor::S)
                    };
                    Some((self.x + 1, self.y))
                }
                Neighbor::SE => {
                    self.next = Some(Neighbor::S);
                    Some((self.x + 1, self.y + 1))
                }
                Neighbor::S => {
                    if self.x == 0 && self.y == 0 {
                        self.next = None;
                    } else if self.x == 0 { 
                        self.next = Some(Neighbor::N);
                    } else {
                        match self.strat {
                            Neighbors::CardinalAndDiagonal => 
                                self.next = Some(Neighbor::SW),
                            Neighbors::Cardinal => 
                                self.next = Some(Neighbor::W)
                        };
                    }
                    Some((self.x, self.y + 1))
                }
                Neighbor::SW => {
                    self.next = Some(Neighbor::W);
                    Some((self.x - 1, self.y + 1))
                }
                Neighbor::W => {
                    if self.y == 0 { 
                        self.next = None;
                    } else {
                        match self.strat {
                            Neighbors::CardinalAndDiagonal => 
                                self.next = Some(Neighbor::NW),
                            Neighbors::Cardinal => 
                                self.next = Some(Neighbor::N)
                        };
                    }
                    Some((self.x - 1, self.y))
                }
                Neighbor::NW => {
                    self.next = Some(Neighbor::N);
                    Some((self.x - 1, self.y - 1))
                }
                Neighbor::N => {
                    match self.strat {
                        Neighbors::CardinalAndDiagonal => 
                            self.next = Some(Neighbor::NE),
                        Neighbors::Cardinal => 
                            self.next = None
                    };
                    Some((self.x, self.y - 1))
                }
                Neighbor::NE => {
                    self.next = None;
                    Some((self.x + 1, self.y - 1))
                }
            }
        } else {
            None
        }
    }
}

/// Cell Id's are an alias
pub type Id = usize;

impl World {

    pub fn new(width: usize, height: usize, cells: Vec<Cell>) ->
           Result<World, String>
    {
        if width * height != cells.len() {
            Err("Width and height do not match number of cells".to_string())
        } else {
            Ok(
                World {
                    width: width,
                    height: height,
                    cells: cells,
                }
            )
        }
    }

    pub fn coords_for(&self, id: Id) -> Option<(usize, usize)>
    {
        if id < self.cells.len() {
            let y = id / self.width;
            let x = id - y * self.width;
            Some((x,y))
        } else {
            None
        }
    }

    pub fn id_at(&self, x: usize, y: usize) -> Option<Id>
    {
        let id = y * self.width + x;
        if id < self.cells.len() {
            Some(id)
        } else {
            None
        }
    }

    pub fn cell_at_mut(&mut self, x: usize, y: usize) -> Option<&mut Cell>
    {
        if let Some(id) = self.id_at(x,y) {
            // unwrap ok because we trust id
            Some(self.cells.get_mut(id).unwrap())
        } else {
            None
        }
    }

    pub fn cell_at(&self, x: usize, y: usize) -> Option<&Cell>
    {
        if let Some(id) = self.id_at(x,y) {
            // unwrap ok because we trust id
            Some(self.cells.get(id).unwrap())
        } else {
            None
        }
    }

    pub fn cell_mut(&mut self, id: Id) -> Option<&mut Cell> 
    {
        self.cells.get_mut(id)
    }

    pub fn cell(&self, id: Id) -> Option<&Cell> 
    {
        self.cells.get(id)
    }

    pub fn iter_neighbor_ids(&self, id: Id, strat: Neighbors) -> Option<NeighborIter> {
        if let Some((x,y)) = self.coords_for(id) {
            Some(NeighborIter::new(x, y, strat))
        } else {
            None
        }
    }

    pub fn width(&self) -> usize { self.width }
    pub fn height(&self) -> usize { self.height }

}

#[cfg(test)]
mod tests {

    use super::*;

    fn mkworld() -> World
    {
        World::new( 4, 4, vec![Cell::Open; 16]).unwrap()
    }

    #[test]
    fn id_at() {
        let uut = mkworld();
        assert_eq!(uut.id_at(0,0), Some(0));
        assert_eq!(uut.id_at(3,0), Some(3));
        assert_eq!(uut.id_at(0,3), Some(12));
        assert_eq!(uut.id_at(1,1), Some(5));
        assert_eq!(uut.id_at(3,3), Some(15));
        assert_eq!(uut.id_at(3,4), None);
        assert_eq!(uut.id_at(4,3), None);
    }

    #[test]
    fn cell_at() {
        let uut = mkworld();
        assert_eq!(*(uut.cell_at(1,1).unwrap()), Cell::Open);
        assert_eq!(uut.cell_at(3,4), None);
    }

    #[test]
    fn neighbor_iter_all() {
        let mut ni = NeighborIter::new(1,1,Neighbors::CardinalAndDiagonal);
        assert_eq!(ni.next().unwrap(), (2,1));
        assert_eq!(ni.next().unwrap(), (2,2));
        assert_eq!(ni.next().unwrap(), (1,2));
        assert_eq!(ni.next().unwrap(), (0,2));
        assert_eq!(ni.next().unwrap(), (0,1));
        assert_eq!(ni.next().unwrap(), (0,0));
        assert_eq!(ni.next().unwrap(), (1,0));
        assert_eq!(ni.next().unwrap(), (2,0));
        assert_eq!(ni.next(), None);

        let mut ni = NeighborIter::new(0,1,Neighbors::CardinalAndDiagonal);
        assert_eq!(ni.next().unwrap(), (1,1));
        assert_eq!(ni.next().unwrap(), (1,2));
        assert_eq!(ni.next().unwrap(), (0,2));
        assert_eq!(ni.next().unwrap(), (0,0));
        assert_eq!(ni.next().unwrap(), (1,0));
        assert_eq!(ni.next(), None);

        let mut ni = NeighborIter::new(1,0,Neighbors::CardinalAndDiagonal);
        assert_eq!(ni.next().unwrap(), (2,0));
        assert_eq!(ni.next().unwrap(), (2,1));
        assert_eq!(ni.next().unwrap(), (1,1));
        assert_eq!(ni.next().unwrap(), (0,1));
        assert_eq!(ni.next().unwrap(), (0,0));
        assert_eq!(ni.next(), None);

        let mut ni = NeighborIter::new(0,0,Neighbors::CardinalAndDiagonal);
        assert_eq!(ni.next().unwrap(), (1,0));
        assert_eq!(ni.next().unwrap(), (1,1));
        assert_eq!(ni.next().unwrap(), (0,1));
        assert_eq!(ni.next(), None);
    }

    #[test]
    fn neighbor_iter_card() {
        let mut ni = NeighborIter::new(1,1,Neighbors::Cardinal);
        assert_eq!(ni.next().unwrap(), (2,1));
        assert_eq!(ni.next().unwrap(), (1,2));
        assert_eq!(ni.next().unwrap(), (0,1));
        assert_eq!(ni.next().unwrap(), (1,0));
        assert_eq!(ni.next(), None);

        let mut ni = NeighborIter::new(0,0,Neighbors::Cardinal);
        assert_eq!(ni.next().unwrap(), (1,0));
        assert_eq!(ni.next().unwrap(), (0,1));
        assert_eq!(ni.next(), None);

        let mut ni = NeighborIter::new(0,1,Neighbors::Cardinal);
        assert_eq!(ni.next().unwrap(), (1,1));
        assert_eq!(ni.next().unwrap(), (0,2));
        assert_eq!(ni.next().unwrap(), (0,0));
        assert_eq!(ni.next(), None);

        let mut ni = NeighborIter::new(1,0,Neighbors::Cardinal);
        assert_eq!(ni.next().unwrap(), (2,0));
        assert_eq!(ni.next().unwrap(), (1,1));
        assert_eq!(ni.next().unwrap(), (0,0));
        assert_eq!(ni.next(), None);
    }
}

