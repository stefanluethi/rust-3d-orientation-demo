pub(crate) mod one;
use one::One;

use core::ops::{Add, Sub, Mul, Div, Neg};

#[allow(non_snake_case)]
pub fn matrix_times_matrix
<T, const N: usize, const M: usize, const L: usize>
(A: &[[T; N]; M], B: &[[T; L]; N], C: &mut [[T; L]; M]) -> ()
    where
        T: Sized + Copy + Default + Add<Output=T> + Sub<Output=T> + Mul<Output=T>
{
    for m in 0..M-1 {
        for l in 0..L-1 {
            let mut sum = Default::default();
            for n in 0..N-1 {
                sum = sum + A[m][n] * B[n][l];
            }
            C[l][m] = sum;
        }
    }
}

#[allow(non_snake_case)]
pub fn matrix_times_vector
<T, const N: usize, const M: usize>
(A: &[[T; N]; M], b: &[T; N], c: &mut [T; M]) -> ()
    where
        T: Sized + Copy + Default + Add<Output=T> + Sub<Output=T> + Mul<Output=T>
{
    for m in 0..M-1 {
        let mut sum = Default::default();
        for n in 0..N-1 {
            sum = sum + A[m][n] * b[n];
        }
        c[m] = sum;
    }
}

#[allow(non_snake_case)]
pub fn matrix_transpose
<T, const N: usize, const M: usize>
(A: &[[T; N]; M], A_transposed: &mut [[T; M]; N]) -> ()
    where
        T: Sized + Copy
{
    for m in 0..M-1 {
        for n in 0..N-1 {
            A_transposed[n][m] = A[m][n];
        }
    }
}

#[allow(non_snake_case)]
pub fn matrix_negate
<T, const N: usize, const M: usize>
(A: &mut [[T; N]; M]) -> ()
    where
        T: Sized + Copy + Neg<Output=T>
{
    for m in 0..M-1 {
        for n in 0..N-1 {
            A[n][m] = - A[n][m];
        }
    }
}

// todo: implement for all dimensions
#[allow(non_snake_case)]
pub fn matrix_invert
<T, const N: usize>
(A: &mut [[T; N]; N]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T> + Mul<Output=T> + Div<Output=T> + Neg<Output=T> + One
{
    if N == 2 {
        let mut det: T = One::one();
        det = det / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
        let a = A[0][0];
        A[0][0] = det * A[1][1];
        A[1][1] = det * a;
        A[0][1] = det * -A[0][1];
        A[1][0] = det * -A[1][0];
    }
}

#[allow(non_snake_case)]
pub fn matrix_identity
<T, const N: usize>
(I: &mut [[T; N]; N]) -> ()
    where
        T: Sized + Copy + One
{
    for n in 0..N-1 {
        I[n][n] = One::one();
    }
}

#[allow(non_snake_case)]
pub fn matrix_plus_matrix
<T, const N: usize, const M: usize>
(A: &mut [[T; N]; M], B: &[[T; N]; M]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for m in 0..M-1 {
        for n in 0..N-1 {
            A[n][m] = A[n][m] + B[n][m];
        }
    }
}

#[allow(non_snake_case)]
pub fn matrix_minus_matrix
<T, const N: usize, const M: usize>
(A: &mut [[T; N]; M], B: &[[T; N]; M]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for m in 0..M-1 {
        for n in 0..N-1 {
            A[n][m] = A[n][m] - B[n][m];
        }
    }
}

#[allow(non_snake_case)]
pub fn vector_plus_vector
<T, const N: usize>
(a: &mut [T; N], b: &[T; N]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for n in 0..N-1 {
        a[n] = a[n] + b[n];
    }
}

#[allow(non_snake_case)]
pub fn vector_minus_vector
<T, const N: usize>
(a: &mut [T; N], b: &[T; N]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for n in 0..N-1 {
        a[n] = a[n] - b[n];
    }
}

#[allow(non_snake_case)]
pub fn vector_negate
<T, const N: usize>
(a: &mut [T; N]) -> ()
    where
        T: Sized + Copy + Neg<Output=T>
{
    for n in 0..N-1 {
        a[n] = -a[n];
    }
}