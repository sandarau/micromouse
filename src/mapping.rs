pub const MICROMOUSE_CELLS: usize = 16 * 16;
pub const INF_COST: u16 = u16::MAX;
pub const CENTER_GOALS_16: [Coord; 4] = [
    Coord::new(7, 7),
    Coord::new(7, 8),
    Coord::new(8, 7),
    Coord::new(8, 8),
];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Coord {
    pub x: u8,
    pub y: u8,
}

impl Coord {
    pub const fn new(x: u8, y: u8) -> Self {
        Self { x, y }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    North,
    East,
    South,
    West,
}

impl Direction {
    pub const ALL: [Direction; 4] = [
        Direction::North,
        Direction::East,
        Direction::South,
        Direction::West,
    ];

    pub const fn left(self) -> Self {
        match self {
            Direction::North => Direction::West,
            Direction::East => Direction::North,
            Direction::South => Direction::East,
            Direction::West => Direction::South,
        }
    }

    pub const fn right(self) -> Self {
        match self {
            Direction::North => Direction::East,
            Direction::East => Direction::South,
            Direction::South => Direction::West,
            Direction::West => Direction::North,
        }
    }

    pub const fn back(self) -> Self {
        match self {
            Direction::North => Direction::South,
            Direction::East => Direction::West,
            Direction::South => Direction::North,
            Direction::West => Direction::East,
        }
    }

    pub const fn bit(self) -> u8 {
        match self {
            Direction::North => 0b0001,
            Direction::East => 0b0010,
            Direction::South => 0b0100,
            Direction::West => 0b1000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FloodFillMode {
    ExploreUnknownAsOpen,
    KnownOpenOnly,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RangeSet {
    pub front_m: Option<f32>,
    pub left_m: Option<f32>,
    pub right_m: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MazeError {
    InvalidDimensions,
    OutOfBounds,
    TooManyGoals,
    QueueFull,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Maze<const CELLS: usize> {
    width: u8,
    height: u8,
    walls: [u8; CELLS],
    known: [u8; CELLS],
}

impl<const CELLS: usize> Maze<CELLS> {
    pub const fn new_unchecked(width: u8, height: u8) -> Self {
        Self {
            width,
            height,
            walls: [0; CELLS],
            known: [0; CELLS],
        }
    }

    pub fn new(width: u8, height: u8) -> Result<Self, MazeError> {
        if width == 0 || height == 0 || width as usize * height as usize > CELLS {
            return Err(MazeError::InvalidDimensions);
        }

        let mut maze = Self::new_unchecked(width, height);
        maze.mark_outer_walls();
        Ok(maze)
    }

    pub fn width(&self) -> u8 {
        self.width
    }

    pub fn height(&self) -> u8 {
        self.height
    }

    pub fn set_wall(
        &mut self,
        cell: Coord,
        direction: Direction,
        present: bool,
    ) -> Result<(), MazeError> {
        let idx = self.index(cell)?;
        self.known[idx] |= direction.bit();

        if present || self.neighbor(cell, direction).is_none() {
            self.walls[idx] |= direction.bit();
        } else {
            self.walls[idx] &= !direction.bit();
        }

        if let Some(other) = self.neighbor(cell, direction) {
            let other_idx = self.index(other)?;
            let opposite = direction.back();
            self.known[other_idx] |= opposite.bit();
            if present {
                self.walls[other_idx] |= opposite.bit();
            } else {
                self.walls[other_idx] &= !opposite.bit();
            }
        }

        Ok(())
    }

    pub fn has_wall(&self, cell: Coord, direction: Direction) -> Result<bool, MazeError> {
        if self.neighbor(cell, direction).is_none() {
            return Ok(true);
        }

        let idx = self.index(cell)?;
        Ok((self.walls[idx] & direction.bit()) != 0)
    }

    pub fn is_known(&self, cell: Coord, direction: Direction) -> Result<bool, MazeError> {
        if self.neighbor(cell, direction).is_none() {
            return Ok(true);
        }

        let idx = self.index(cell)?;
        Ok((self.known[idx] & direction.bit()) != 0)
    }

    pub fn update_from_ranges(
        &mut self,
        cell: Coord,
        heading: Direction,
        ranges: RangeSet,
        wall_threshold_m: f32,
    ) -> Result<(), MazeError> {
        if let Some(distance) = ranges.front_m {
            self.set_wall(cell, heading, distance <= wall_threshold_m)?;
        }
        if let Some(distance) = ranges.left_m {
            self.set_wall(cell, heading.left(), distance <= wall_threshold_m)?;
        }
        if let Some(distance) = ranges.right_m {
            self.set_wall(cell, heading.right(), distance <= wall_threshold_m)?;
        }

        Ok(())
    }

    pub fn flood_fill<const GOALS: usize>(
        &self,
        goals: &[Coord; GOALS],
        goal_count: usize,
        costs: &mut [u16; CELLS],
        mode: FloodFillMode,
    ) -> Result<(), MazeError> {
        if goal_count > GOALS {
            return Err(MazeError::TooManyGoals);
        }

        let used = self.used_cells();
        for cost in costs.iter_mut().take(used) {
            *cost = INF_COST;
        }

        let mut queue = FixedQueue::<CELLS>::new();
        for goal in goals.iter().take(goal_count) {
            let idx = self.index(*goal)?;
            costs[idx] = 0;
            queue.push(*goal)?;
        }

        while let Some(cell) = queue.pop() {
            let base = costs[self.index(cell)?];
            for direction in Direction::ALL {
                if !self.can_move(cell, direction, mode)? {
                    continue;
                }

                let Some(next) = self.neighbor(cell, direction) else {
                    continue;
                };

                let next_idx = self.index(next)?;
                let next_cost = base.saturating_add(1);
                if next_cost < costs[next_idx] {
                    costs[next_idx] = next_cost;
                    queue.push(next)?;
                }
            }
        }

        Ok(())
    }

    pub fn best_next_direction(
        &self,
        costs: &[u16; CELLS],
        cell: Coord,
        heading: Direction,
        mode: FloodFillMode,
    ) -> Result<Option<Direction>, MazeError> {
        let preferences = [heading, heading.left(), heading.right(), heading.back()];
        let mut best_direction = None;
        let mut best_cost = INF_COST;

        for direction in preferences {
            if !self.can_move(cell, direction, mode)? {
                continue;
            }

            let Some(next) = self.neighbor(cell, direction) else {
                continue;
            };

            let cost = costs[self.index(next)?];
            if cost < best_cost {
                best_cost = cost;
                best_direction = Some(direction);
            }
        }

        Ok(best_direction)
    }

    pub fn neighbor(&self, cell: Coord, direction: Direction) -> Option<Coord> {
        match direction {
            Direction::North if cell.y + 1 < self.height => Some(Coord::new(cell.x, cell.y + 1)),
            Direction::East if cell.x + 1 < self.width => Some(Coord::new(cell.x + 1, cell.y)),
            Direction::South if cell.y > 0 => Some(Coord::new(cell.x, cell.y - 1)),
            Direction::West if cell.x > 0 => Some(Coord::new(cell.x - 1, cell.y)),
            _ => None,
        }
    }

    fn can_move(
        &self,
        cell: Coord,
        direction: Direction,
        mode: FloodFillMode,
    ) -> Result<bool, MazeError> {
        if self.neighbor(cell, direction).is_none() || self.has_wall(cell, direction)? {
            return Ok(false);
        }

        match mode {
            FloodFillMode::ExploreUnknownAsOpen => Ok(true),
            FloodFillMode::KnownOpenOnly => self.is_known(cell, direction),
        }
    }

    fn index(&self, cell: Coord) -> Result<usize, MazeError> {
        if cell.x >= self.width || cell.y >= self.height {
            return Err(MazeError::OutOfBounds);
        }
        Ok(cell.y as usize * self.width as usize + cell.x as usize)
    }

    fn used_cells(&self) -> usize {
        self.width as usize * self.height as usize
    }

    fn mark_outer_walls(&mut self) {
        for x in 0..self.width {
            let _ = self.set_wall(Coord::new(x, 0), Direction::South, true);
            let _ = self.set_wall(Coord::new(x, self.height - 1), Direction::North, true);
        }
        for y in 0..self.height {
            let _ = self.set_wall(Coord::new(0, y), Direction::West, true);
            let _ = self.set_wall(Coord::new(self.width - 1, y), Direction::East, true);
        }
    }
}

struct FixedQueue<const CAP: usize> {
    data: [Coord; CAP],
    head: usize,
    tail: usize,
    len: usize,
}

impl<const CAP: usize> FixedQueue<CAP> {
    const fn new() -> Self {
        Self {
            data: [Coord::new(0, 0); CAP],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    fn push(&mut self, value: Coord) -> Result<(), MazeError> {
        if self.len == CAP {
            return Err(MazeError::QueueFull);
        }

        self.data[self.tail] = value;
        self.tail = (self.tail + 1) % CAP;
        self.len += 1;
        Ok(())
    }

    fn pop(&mut self) -> Option<Coord> {
        if self.len == 0 {
            return None;
        }

        let value = self.data[self.head];
        self.head = (self.head + 1) % CAP;
        self.len -= 1;
        Some(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flood_fill_guides_start_toward_center() {
        let maze = Maze::<MICROMOUSE_CELLS>::new(16, 16).unwrap();
        let mut costs = [INF_COST; MICROMOUSE_CELLS];

        maze.flood_fill(
            &CENTER_GOALS_16,
            CENTER_GOALS_16.len(),
            &mut costs,
            FloodFillMode::ExploreUnknownAsOpen,
        )
        .unwrap();

        let next = maze
            .best_next_direction(
                &costs,
                Coord::new(0, 0),
                Direction::North,
                FloodFillMode::ExploreUnknownAsOpen,
            )
            .unwrap();

        assert_eq!(next, Some(Direction::North));
    }
}
