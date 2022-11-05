// use num_traits::{Float, Inv};

use super::traits::Sqrt;

/// Helper struct represent a 3D point with plane and linear
///
/// plane is (item.0,item.1), linear is item.2

#[derive(Eq, PartialEq, Copy, Clone, Debug, Default)]
/// Helper struct defining a 2D point in space.
pub struct Vec3<T>(pub T, pub T, pub T);
impl<T> Vec3<T>
where
    T: Copy
        + core::cmp::PartialOrd
        + num_traits::Zero
        + num_traits::One
        + core::ops::Add<Output = T>
        + core::ops::Sub<Output = T>
        + core::ops::Mul<Output = T>
        + core::ops::Neg<Output = T>
        + core::ops::Div<Output = T>, // + Ceil
{
    /// Create a new point from an x/y/z coordinate.
    pub fn new(axis0: T, axis1: T, axis2: T) -> Self {
        Self {
            0: axis0,
            1: axis1,
            2: axis2,
        }
    }
    pub fn zero() -> Self {
        Self {
            0: T::zero(),
            1: T::zero(),
            2: T::zero(),
        }
    }
    #[inline]
    pub fn new_from_plane(plane: Vec2<T>, linear: T) -> Self {
        Self {
            0: plane.0,
            1: plane.1,
            2: linear,
        }
    }

    #[inline]
    pub fn dot(&self, other: Self) -> T {
        (self.0 * other.0) + (self.1 * other.1) + (self.2 * other.2)
    }
    // #[inline]
    // pub fn cross(&self, other: Self) -> Self {
    //     Self {
    //         0: (self.y * other.z) + (-self.z * other.y),
    //         1: (self.z * other.x) + (-self.x * other.z),
    //         2: (self.x * other.y) + (-self.y * other.x),
    //     }
    // }

    pub fn abs(&self) -> Self {
        Self {
            0: if self.0 < T::zero() {
                self.0.neg()
            } else {
                self.0
            },
            1: if self.1 < T::zero() {
                self.1.neg()
            } else {
                self.1
            },
            2: if self.2 < T::zero() {
                self.2.neg()
            } else {
                self.2
            },
        }
    }
    pub fn max_element(&self) -> T {
        let t = if self.0 > self.1 { self.0 } else { self.1 };

        let tt = if t > self.2 { t } else { self.2 };

        tt
    }
    pub fn max_element_index(&self) -> Self {
        let mut unit = Self::new(T::one(), T::zero(), T::zero());
        let t = if self.0 > self.1 {
            self.0
        } else {
            unit = Self::new(T::zero(), T::one(), T::zero());
            self.1
        };

        if t > self.2 {
        } else {
            unit = Self::new(T::zero(), T::zero(), T::one());
        };

        unit
    }

    /// x^2+y^2+z^2
    #[inline]
    pub fn distance_sqr(&self) -> T
    where
        T: core::ops::Mul<Output = T>,
    {
        self.0 * self.0 + self.1 * self.1 + self.2 * self.2
    }

    /// \sqrt{x^2+y^2+z^2}
    #[inline]
    pub fn distance(&self) -> T
    where
        T: core::ops::Mul<Output = T> + Sqrt,
    {
        self.distance_sqr().sqrt()
    }

    /// unit vec
    pub fn as_unit_vec(&self) -> Self
    where
        T: core::ops::Mul<Output = T> + num_traits::Inv<Output = T> + Sqrt,
    {
        let magnitude = self.distance().inv();
        Self {
            0: self.0 * magnitude,
            1: self.1 * magnitude,
            2: self.2 * magnitude,
        }
    }
    #[inline]
    pub fn plane(&self) -> Vec2<T> {
        Vec2 {
            0: self.0,
            1: self.1,
        }
    }

    #[inline]
    pub fn linear(&self) -> T {
        self.2
    }
}

impl<T> core::ops::Sub for Vec3<T>
where
    T: Copy + core::ops::Sub<Output = T>,
{
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self::Output) -> Self {
        Self {
            0: self.0 - rhs.0,
            1: self.1 - rhs.1,
            2: self.2 - rhs.2,
        }
    }
}

impl<T> core::ops::Add for Vec3<T>
where
    T: Copy + core::ops::Add<Output = T>,
{
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self::Output) -> Self {
        Self {
            0: self.0 + rhs.0,
            1: self.1 + rhs.1,
            2: self.2 + rhs.2,
        }
    }
}

impl<T> core::ops::Mul<T> for &Vec3<T>
where
    T: Copy + core::ops::Mul<Output = T>,
{
    type Output = Vec3<T>;
    #[inline]
    fn mul(self, rhs: T) -> Self::Output {
        Self::Output {
            0: self.0 * rhs,
            1: self.1 * rhs,
            2: self.2 * rhs,
        }
    }
}

impl<T> core::ops::Div<T> for &Vec3<T>
where
    T: Copy + core::ops::Div<Output = T>,
{
    type Output = Vec3<T>;
    #[inline]
    fn div(self, rhs: T) -> Self::Output {
        Self::Output {
            0: self.0 / rhs,
            1: self.1 / rhs,
            2: self.2 / rhs,
        }
    }
}

impl<T> core::ops::Neg for Vec3<T>
where
    T: core::ops::Neg<Output = T>,
{
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            0: -self.0,
            1: -self.1,
            2: -self.2,
        }
    }
}

pub type Point = Vec2<i32>;
pub type Point3 = Vec3<i32>;

#[derive(Eq, PartialEq, Copy, Clone, Debug, Default)]
/// Helper struct defining a 2D point in space.
pub struct Vec2<T>(pub T, pub T);
impl<T> Vec2<T>
where
    T: Copy
        + core::cmp::PartialOrd
        + num_traits::Zero
        + num_traits::One
        + core::ops::Add<Output = T>
        + core::ops::Sub<Output = T>
        + core::ops::Mul<Output = T>
        + core::ops::Neg<Output = T>
        + core::ops::Div<Output = T>, // + Ceil
{
    /// Create a new point from an x/y/z coordinate.
    pub fn new(axis0: T, axis1: T) -> Self {
        Self { 0: axis0, 1: axis1 }
    }

    /// The wedge (aka exterior) product of two vectors.
    ///
    /// Note: Sometimes called "cross" product in 2D.
    /// Such a product is not well defined in 2 dimensions
    /// and is really just shorthand notation for a hacky operation that
    /// extends the vectors into 3 dimensions, takes the cross product,
    /// then returns only the resulting Z component as a pseudoscalar value.
    /// This value is will have the same value as
    /// the resulting bivector of the wedge product in 2d (a 2d
    /// bivector is also a kind of pseudoscalar value), so you may use
    /// this product to calculate the same value.
    ///
    /// This operation results in a bivector, which represents
    /// the plane parallel to the two vectors, and which has a
    /// 'oriented area' equal to the parallelogram created by extending
    /// the two vectors, oriented such that the positive direction is the
    /// one which would move `self` closer to `other`.
    #[inline]
    pub fn wedge(&self, other: Self) -> T {
        (self.0 * other.1) - (self.1 * other.0)
    }

    #[inline]
    pub fn dot(&self, other: Self) -> T {
        (self.0 * other.0) + (self.1 * other.1)
    }

    /// x^2+y^2+z^2
    #[inline]
    pub fn distance_sqr(&self) -> T
    where
        T: core::ops::Mul<Output = T>,
    {
        self.0 * self.0 + self.1 * self.1
    }

    /// \sqrt{x^2+y^2+z^2}
    #[inline]
    pub fn distance(&self) -> T
    where
        T: Sqrt,
    {
        self.distance_sqr().sqrt()
    }

    /// unit vec
    pub fn as_unit_vec(&self) -> Self
    where
        T: core::ops::Mul<Output = T> + num_traits::Inv<Output = T> + Sqrt,
    {
        let magnitude = self.distance().inv();
        Self {
            0: self.0 * magnitude,
            1: self.1 * magnitude,
        }
    }
}

impl<T> core::ops::Sub for Vec2<T>
where
    T: Copy + core::ops::Sub<Output = T>,
{
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self::Output) -> Self {
        Self {
            0: self.0 - rhs.0,
            1: self.1 - rhs.1,
        }
    }
}

impl<T> core::ops::Add for Vec2<T>
where
    T: Copy + core::ops::Add<Output = T>,
{
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self::Output) -> Self {
        Self {
            0: self.0 + rhs.0,
            1: self.1 + rhs.1,
        }
    }
}

impl<T> core::ops::Neg for Vec2<T>
where
    T: core::ops::Neg<Output = T>,
{
    type Output = Self;
    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            0: -self.0,
            1: -self.1,
        }
    }
}

#[allow(dead_code)]
pub struct Vec5<T>(pub T, pub T, pub T, pub T, pub T);
impl<T> Vec5<T>
where
    T: Copy,
{
    /// Create a new point from an x/y/z coordinate.
    #[allow(dead_code)]
    pub fn new(axis0: T, axis1: T, axis2: T, axis3: T, axis4: T) -> Self {
        Self {
            0: axis0,
            1: axis1,
            2: axis2,
            3: axis3,
            4: axis4,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CanonPlane {
    /// Z-axis, XY-plane
    CanonPlaneXY = 1,
    /// X-axis, YZ-plane
    CanonPlaneYZ = 2,
    /// Y-axis, XZ-plane
    CanonPlaneXZ = 3,
}

impl CanonPlane {
    /// data based on self plane convert to new data that based on dest plane
    pub fn to_plane<T>(&self, orig: &Vec3<T>, dest_plane: &CanonPlane) -> Vec3<T>
    where
        T: Copy + Clone,
    {
        if self == dest_plane {
            return (*orig).clone();
        }

        let (x, y, z) = if self == &Self::CanonPlaneYZ {
            (orig.2, orig.0, orig.1)
        } else if self == &Self::CanonPlaneXZ {
            (orig.1, orig.2, orig.0)
        } else {
            (orig.0, orig.1, orig.2)
        };
        let dest = if dest_plane == &Self::CanonPlaneYZ {
            Vec3::<T> { 0: y, 1: z, 2: x }
        } else if dest_plane == &Self::CanonPlaneXZ {
            Vec3::<T> { 0: z, 1: x, 2: y }
        } else {
            Vec3::<T> { 0: x, 1: y, 2: z }
        };
        dest
    }
}
