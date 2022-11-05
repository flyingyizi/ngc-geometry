//! [The Beauty of Line2D's Algorithm](http://members.chello.at/~easyfilter/bresenham.html)

// use core::ops;

use super::vecx::{Point, Point3, Vec3};

// use num_traits::{clamp_max, clamp_min};

// pub type DefaultNum = fixed::FixedU64<typenum::U32>;

struct Octant(u8);

impl Octant {
    /// output end-p belongs to which octant of start-p
    #[allow(dead_code)]
    #[inline]
    fn from_points(start: Point, end: Point) -> Octant {
        let mut dx = end.0 - start.0;
        let mut dy = end.1 - start.1;

        let mut octant = 0;

        // rotate by 180 degrees
        if dy < 0 {
            dx = -dx;
            dy = -dy;
            octant += 4;
        }
        // rotate clockwise by 90 degrees
        if dx < 0 {
            let tmp = dx;
            dx = dy;
            dy = -tmp;
            octant += 2
        }
        // no need to rotate now
        if dx < dy {
            octant += 1
        }

        Octant(octant)
    }

    /// p as cotant-0
    #[allow(dead_code)]
    #[inline]
    fn to_octant0(&self, p: Point) -> Point {
        match self.0 {
            0 => Point::new(p.0, p.1),
            1 => Point::new(p.1, p.0),
            2 => Point::new(p.1, -p.0),
            3 => Point::new(-p.0, p.1),
            4 => Point::new(-p.0, -p.1),
            5 => Point::new(-p.1, -p.0),
            6 => Point::new(-p.1, p.0),
            7 => Point::new(p.0, -p.1),
            _ => unreachable!(),
        }
    }

    #[inline]
    fn from_octant0(&self, p: Point) -> Point {
        match self.0 {
            0 => Point::new(p.0, p.1),
            1 => Point::new(p.1, p.0),
            2 => Point::new(-p.1, p.0),
            3 => Point::new(-p.0, p.1),
            4 => Point::new(-p.0, -p.1),
            5 => Point::new(-p.1, -p.0),
            6 => Point::new(p.1, -p.0),
            7 => Point::new(p.0, -p.1),
            _ => unreachable!(),
        }
    }
}

pub struct Line2D {
    // point
    x: i32,
    y: i32,

    dx: i32,
    dy: i32,
    x1: i32,
    diff: i32,
    octant: Octant,
}

impl Line2D {
    /// Creates a new iterator.Yields intermediate points between `start`
    /// and `end`. Does include `start` but not `end`.
    #[allow(dead_code)]
    #[inline]
    pub fn new(start: Point, end: Point) -> Line2D {
        let octant = Octant::from_points(start, end);

        let start = octant.to_octant0(start);
        let end = octant.to_octant0(end);

        let dx = end.0 - start.0;
        let dy = end.1 - start.1;

        Line2D {
            x: start.0,
            y: start.1,
            dx,
            dy,
            x1: end.0,
            diff: dy - dx,
            octant,
        }
    }

    /// return how much points in the line
    #[allow(dead_code)]
    pub fn len(&self) -> i32 {
        return self.x1 - self.x;
    }
}

impl core::iter::Iterator for Line2D {
    type Item = Point;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.x >= self.x1 {
            return None;
        }

        let p = Point::new(self.x, self.y);

        if self.diff >= 0 {
            self.y += 1;
            self.diff -= self.dx;
        }

        self.diff += self.dy;

        // loop inc
        self.x += 1;

        Some(self.octant.from_octant0(p))
    }
}

// void plotLine3d(int x0, int y0, int z0, int x1, int y1, int z1)
// {
//    int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
//    int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
//    int dz = abs(z1-z0), sz = z0<z1 ? 1 : -1;
//    int dm = max(dx,dy,dz), i = dm; /* maximum difference */
//    x1 = y1 = z1 = dm/2; /* error offset */
//    for(;;) {  /* loop */
//       setPixel(x0,y0,z0);
//       if (i-- == 0) break;
//       x1 -= dx; if (x1 < 0) { x1 += dm; x0 += sx; }
//       y1 -= dy; if (y1 < 0) { y1 += dm; y0 += sy; }
//       z1 -= dz; if (z1 < 0) { z1 += dm; z0 += sz; }
//    }
// }
#[derive(Copy, Clone, Debug)]
/// line3d, in its core::iter::Iterator implement, all output points include end point, not include start point
pub struct Line3D {
    /// absolute length on each dimensions
    d: Vec3<i32>,
    ///step 1 or -1
    s: Vec3<i32>,
    /// maximum difference
    dm: i32,

    //var
    out: Vec3<i32>,
    i: i32,
    /// error offset
    err_oft: Vec3<i32>,
}

impl Line3D {
    /// Creates a new iterator.Yields intermediate points between `start`
    /// and `end`. Does include `start` but not `end`.
    ///
    /// the unit in the position is based on the Maximum resolution. e.g. if
    /// the Maximum resolution is 600X800, then x \in [0,599], y \in [0,799]
    ///
    #[inline]
    pub fn new(start: Point3, end: Point3) -> Self {
        let s: Vec3<i32> = Vec3::new(
            if start.0 < end.0 { 1 } else { -1 },
            if start.1 < end.1 { 1 } else { -1 },
            if start.2 < end.2 { 1 } else { -1 },
        );
        let dm = (end - start).abs().max_element();
        let err_oft: Vec3<i32> = Vec3::new(dm / 2, dm / 2, dm / 2);

        Self {
            d: (end - start).abs(),
            s,
            dm,
            i: dm,

            out: start,
            err_oft,
        }
    }
    pub fn len(&self) -> usize {
        self.dm as usize
    }
}

impl core::iter::Iterator for Line3D {
    type Item = Point3;

    /// all output point include end point, not include start point
    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        // algorithm:
        // for(;;) {  /* loop */
        //     setPixel(x0,y0,z0);
        //     if (i-- == 0) break;
        //     x1 -= dx; if (x1 < 0) { x1 += dm; x0 += sx; }
        //     y1 -= dy; if (y1 < 0) { y1 += dm; y0 += sy; }
        //     z1 -= dz; if (z1 < 0) { z1 += dm; z0 += sz; }
        //  }
        if self.i == 0 {
            return None;
        }

        self.err_oft = self.err_oft - self.d;
        if self.err_oft.0 < 0 {
            self.err_oft.0 += self.dm;
            self.out.0 += self.s.0;
        }
        if self.err_oft.1 < 0 {
            self.err_oft.1 += self.dm;
            self.out.1 += self.s.1;
        }
        if self.err_oft.2 < 0 {
            self.err_oft.2 += self.dm;
            self.out.2 += self.s.2;
        }

        let p = self.out.clone();
        // loop inc
        self.i -= 1;
        Some(p)
    }
}

#[cfg(test)]
mod tests {
    use super::{Line2D, Line3D, Point, Point3};

    #[test]
    fn test_line2d_example() {
        // let bi = Line2D::new(Point::new(6, 4), Point::new(0, 1));
        let bi = Line2D::new(Point::new(0, 1), Point::new(6, 4));
        assert_eq!(bi.len(), 6);

        let res: Vec<_> = bi.collect();

        // let slice = &[1, 2, 3];
        // let vec: Vec<i32, 4> = slice.iter().cloned().collect();
        // assert_eq!(&vec, slice);
        assert_eq!(
            res,
            [
                Point::new(0, 1),
                Point::new(1, 1),
                Point::new(2, 2),
                Point::new(3, 2),
                Point::new(4, 3),
                Point::new(5, 3),
            ]
        )
    }
    #[test]
    fn test_line3d_example() {
        // let bi = Line2D::new(Point::new(6, 4), Point::new(0, 1));
        let bi = Line3D::new(Point3::new(0, 0, 0), Point3::new(6, 6, 6));
        assert_eq!(bi.len(), 6);

        let res: Vec<_> = bi.collect();
        assert_eq!(
            res,
            [
                Point3::new(1, 1, 1),
                Point3::new(2, 2, 2),
                Point3::new(3, 3, 3),
                Point3::new(4, 4, 4),
                Point3::new(5, 5, 5),
                Point3::new(6, 6, 6),
            ]
        );

        let bi = Line3D::new(Point3::new(0, 0, 0), Point3::new(0, 0, 6));
        assert_eq!(bi.len(), 6);

        let res: Vec<_> = bi.collect();
        assert_eq!(
            res,
            [
                Point3::new(0, 0, 1),
                Point3::new(0, 0, 2),
                Point3::new(0, 0, 3),
                Point3::new(0, 0, 4),
                Point3::new(0, 0, 5),
                Point3::new(0, 0, 6),
            ]
        );

        let bi = Line3D::new(Point3::new(6, 4, 0), Point3::new(0, 1, 0));
        let res: Vec<_> = bi.collect();

        assert_eq!(
            res,
            [
                Point3::new(5, 4, 0),
                Point3::new(4, 3, 0),
                Point3::new(3, 3, 0),
                Point3::new(2, 2, 0),
                Point3::new(1, 2, 0),
                Point3::new(0, 1, 0),
            ]
        )
    }
}
