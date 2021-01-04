/// Some generic math functions for vectors and matrices

pub(crate) mod one;

#[allow(non_snake_case, dead_code)]
pub mod linalg {
use super::one::One;
use core::ops::{Add, Sub, Mul, Div, Neg};

pub fn matrix_times_matrix
<T, const N: usize, const M: usize, const L: usize>
(A: &[[T; N]; M], B: &[[T; L]; N], C: &mut [[T; L]; M]) -> ()
    where
        T: Sized + Copy + Default + Add<Output=T> + Sub<Output=T> + Mul<Output=T>
{
    for m in 0..M {
        for l in 0..L {
            let mut sum = Default::default();
            for n in 0..N {
                sum = sum + A[m][n] * B[n][l];
            }
            C[m][l] = sum;
        }
    }
}

pub fn matrix_times_vector
<T, const N: usize, const M: usize>
(A: &[[T; N]; M], b: &[T; N], c: &mut [T; M]) -> ()
    where
        T: Sized + Copy + Default + Add<Output=T> + Sub<Output=T> + Mul<Output=T>
{
    for m in 0..M {
        let mut sum = Default::default();
        for n in 0..N {
            sum = sum + A[m][n] * b[n];
        }
        c[m] = sum;
    }
}

pub fn matrix_transpose
<T, const N: usize, const M: usize>
(A: &[[T; N]; M], A_transposed: &mut [[T; M]; N]) -> ()
    where
        T: Sized + Copy
{
    for m in 0..M {
        for n in 0..N {
            A_transposed[n][m] = A[m][n];
        }
    }
}

pub fn matrix_negate
<T, const N: usize, const M: usize>
(A: &mut [[T; N]; M]) -> ()
    where
        T: Sized + Copy + Neg<Output=T>
{
    for m in 0..M {
        for n in 0..N {
            A[n][m] = - A[n][m];
        }
    }
}

// todo: implement for all dimensions
pub fn matrix_invert
<T, const N: usize>
(A: &mut [[T; N]; N]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T> + Mul<Output=T> + Div<Output=T> + Neg<Output=T> + One
{
    if N == 1 {
        let one: T = One::one();
        A[0][0] = one / A[0][0];
    } else if N == 2 {
        let mut det: T = One::one();
        det = det / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
		let a = A[0][0];
        A[0][0] = det * A[1][1];
        A[1][1] = det * a;
        A[0][1] = det * -A[0][1];
        A[1][0] = det * -A[1][0];
    }
}

pub fn matrix_identity
<T, const N: usize>
(I: &mut [[T; N]; N]) -> ()
    where
        T: Sized + Copy + One + Default
{
    for n in 0..N {
		for m in 0..N {
			if n == m {
        		I[n][m] = One::one();
			} else {
				I[n][m] = Default::default(); 
			}
		}
    }
}

pub fn matrix_plus_matrix
<T, const N: usize, const M: usize>
(A: &mut [[T; N]; M], B: &[[T; N]; M]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for m in 0..M {
        for n in 0..N {
            A[n][m] = A[n][m] + B[n][m];
			
        }
    }
}

pub fn matrix_minus_matrix
<T, const N: usize, const M: usize>
(A: &mut [[T; N]; M], B: &[[T; N]; M]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for m in 0..M {
        for n in 0..N {
            A[n][m] = A[n][m] - B[n][m];
        }
    }
}

pub fn vector_plus_vector
<T, const N: usize>
(a: &mut [T; N], b: &[T; N]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for n in 0..N {
        a[n] = a[n] + b[n];
    }
}

pub fn vector_minus_vector
<T, const N: usize>
(a: &mut [T; N], b: &[T; N]) -> ()
    where
        T: Sized + Copy + Add<Output=T> + Sub<Output=T>
{
    for n in 0..N {
        a[n] = a[n] - b[n];
    }
}

pub fn vector_negate
<T, const N: usize>
(a: &mut [T; N]) -> ()
    where
        T: Sized + Copy + Neg<Output=T>
{
    for n in 0..N {
        a[n] = -a[n];
    }
}
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::linalg::*;
    use core::mem::MaybeUninit;

    #[test]
     fn test_matrix_times_matrix() {
         let a = [[1., 2.], [3., 4.]];
         let b = [[0., 1.], [1., 0.]];
         let mut c: [[f32; 2]; 2];
         unsafe {
             c = MaybeUninit::uninit().assume_init();
         }
    
         matrix_times_matrix(&a, &b, &mut c);
         assert_eq!(c, [[2., 1.], [4., 3.]]);
     }

     #[test]
     fn test_matrix_times_vector() {
         let a = [[1., 2.], [3., 4.]];
         let b = [0.5, 1.];
         let mut c: [f32; 2];
         unsafe {
             c = MaybeUninit::uninit().assume_init();
         }
    
         matrix_times_vector(&a, &b, &mut c);
         assert_eq!(c, [2.5, 5.5]);
     }

     #[test]
     fn test_matrix_transpose() {
         let a = [[1., 2.], [3., 4.]];
         let mut b: [[f32; 2]; 2];
         unsafe {
             b = MaybeUninit::uninit().assume_init();
         }
         matrix_transpose(&a, &mut b);
         assert_eq!(b, [[1., 3.], [2., 4.]]);
     }

     #[test]
     fn test_matrix_negate() {
         let mut a = [[1., 2.], [3., 4.]];
    
         matrix_negate(&mut a);
         assert_eq!(a, [[-1., -2.], [-3., -4.]]);
     }

     #[test]
     fn test_matrix_invert() {
         let mut a = [[1., 2.], [3., 4.]];
    
         matrix_invert(&mut a);
         assert_eq!(a, [[-2., 1.], [1.5, -0.5]]);
     }

     #[test]
	 //#[ignore]
     fn test_matrix_identity() {
         let mut a: [[f32; 2]; 2];
         unsafe {
             a = MaybeUninit::uninit().assume_init();
         }
   		 let i: [[f32; 2]; 2] = [[1., 0.], [0., 1.]]; 
         matrix_identity(&mut a);
         assert_eq!(a, i);
	}

     #[test]
     fn test_matrix_plus_matrix() {
         let mut a = [[1., 2.], [3., 4.]];
         let b = [[0., 1.], [1., 0.]];
    
         matrix_plus_matrix(&mut a, &b);
         assert_eq!(a, [[1., 3.], [4., 4.]]);
     }

     #[test]
     fn test_matrix_minus_matrix() {
         let mut a = [[1., 2.], [3., 4.]];
         let b = [[0., 1.], [1., 0.]];
    
         matrix_minus_matrix(&mut a, &b);
         assert_eq!(a, [[1., 1.], [2., 4.]]);
     }

     #[test]
     fn test_vector_plus_vector() {
         let mut a = [1., 2.];
         let b = [1., 3.];
    
         vector_plus_vector(&mut a, &b);
         assert_eq!(a, [2., 5.]);
     }

     #[test]
     fn test_vector_minus_vector() {
         let mut a = [1., 2.];
         let b = [1., 3.];
    
         vector_minus_vector(&mut a, &b);
         assert_eq!(a, [0., -1.]);
     }

     #[test]
     fn test_vector_negate() {
         let mut a = [1., 2.];
    
         vector_negate(&mut a);
         assert_eq!(a, [-1., -2.]);
     }
}
