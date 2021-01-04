#![no_std]
#![feature(const_generics)]

mod math;
use math::{one::One, linalg};

use core::ops::{Add, Sub, Mul, Div, Neg};
use core::mem::MaybeUninit;

/// # Generic Kalman filter
/// *Note: this crate very generic and extremely inefficient!*
///
/// ## Nightly Features
/// ### Const Generics
/// A generic Kalman filter must handle vectors and matrices of different sizes.
/// Const generic is the only way to verify consistent dimensions at compile
/// time.
///
/// ## Theory
/// ### Model parameters
/// | Parameter | Dimensions     | Description |
/// | --------- | -------------- | ----------- |
/// | $`x`$     | $`p \cross 1`$ | State vector containing $`p`$ states |
/// | $`y`$     | $`l \cross 1`$ | Measurement vector containing $`l`$ measurements |
/// | $`A`$     | $`p \cross p`$ | State transition matrix, describing systems behaviour |
/// | $`C`$     | $`l \cross p`$ | Observation matrix, describing relation between measurements and the system |
/// | $`Q_w`$   | $`p \cross p`$ | System covariance matrix |
/// | $`Q_v`$   | $`l \cross l`$ | Measurement covariance matrix |
/// | $`P`$     | $`p \cross p`$ | Error covariance matrix: estimation of the current error |
/// | $`K`$     | $`p \cross l`$ | Kalman gain, minimizes mean squared error on every time-step |
///
///
/// ### Discrete Algorithm
/// ### Prediction Step
/// ```math
/// \hat{\vec{x}}_{prior}[n] = A[n-1] \cdot \hat{\vec{x}}[n-1]\\
/// P[n]_{prior} = A[n-1] \cdot P[n-1] \cdot A^H[n-1] + Q_w[n]
/// ```
///
/// ### Correction Step
/// ```math
/// K[n] = P_{prior}[n] \cdot C^H[n] \cdot \left[ C[n] \cdot P_{prior}[n] C^H[n] + Q_v[n] \right]^{-1}\\
/// \hat{\vec{x}}[n] = \hat{\vec{x}}_{prior}[n] + K[n] \cdot \left[ \vec{y}[n] - C[n] \cdot \hat{\vec{x}}_{prior}[n] \right]\\
/// P[n] = \left[ I - K[n] \cdot C[n] \right] \cdot P_{prior}[n]
/// ```
/// Our main interest is on the state estimation $`\hat{\vec{x}}`$ and error
/// covariance $`P`$ which indicates the certainty of the state estimation.
///
/// ## Reference
/// M. H. Hayes, "Statistical Digital Signal Processing and Modeling"

#[allow(non_snake_case, non_upper_case_globals)]
pub struct Kalman<T, const p: usize, const l: usize>
    where
        T: Sized + Copy + Default + Add<Output=T> + Sub<Output=T> + Mul<Output=T> + Div<Output=T>,
{
    x: [T; p],
    A: [[T; p]; p],
    C: [[T; p]; l],
    Q_w: [[T; p]; p],
    Q_v: [[T; l]; l],
    P: [[T; p]; p],
    K: [[T; l]; p],
}

#[allow(non_snake_case, non_upper_case_globals)]
impl <T, const p: usize, const l: usize> Kalman<T, {p}, {l}>
    where
        T: Sized + Copy + Default + Add<Output=T> + Sub<Output=T> + Mul<Output=T> + Neg<Output=T> + Div<Output=T> + One
{
    pub fn new(x: [T; p],
               A: [[T; p]; p],
               C: [[T; p]; l],
               Q_w: [[T; p]; p],
               Q_v: [[T; l]; l],
               P: [[T; p]; p],
               K:[[T; l]; p]) -> Self {
        let kalman = Kalman {
            x: x,
            A: A,
            C: C,
            Q_w: Q_w,
            Q_v: Q_v,
            P: P,
            K: K
        };

        kalman
    }

    pub fn update_step(&mut self, y: [T; l]) -> () {
        //let mut x_prior: [T; p] = [Default::default(); p]; // not possible yet
        let mut x_prior: [T; p];
        let mut P_prior: [[T; p]; p];

        let mut At: [[T; p]; p];
        let mut Ct: [[T; l]; p];
        let mut PAt: [[T; p]; p];
        let mut PCt: [[T; l]; p];
        let mut CPCtQ: [[T; l]; l];
        let mut CtCPCtQ: [[T; l]; p];
        let mut vec_l: [T; l];
        let mut vec_p: [T; p];
        let mut KC: [[T; p]; p];
        let mut I_p: [[T; p]; p];
        unsafe {
            x_prior = MaybeUninit::uninit().assume_init();
            P_prior = MaybeUninit::uninit().assume_init();
            At = MaybeUninit::uninit().assume_init();
            Ct = MaybeUninit::uninit().assume_init();
            PAt = MaybeUninit::uninit().assume_init();
            PCt = MaybeUninit::uninit().assume_init();
            CPCtQ = MaybeUninit::uninit().assume_init();
            CtCPCtQ = MaybeUninit::uninit().assume_init();
            vec_l = MaybeUninit::uninit().assume_init();
            vec_p = MaybeUninit::uninit().assume_init();
            KC = MaybeUninit::uninit().assume_init();
            I_p = MaybeUninit::uninit().assume_init();
        }

        linalg::matrix_transpose(&self.A, &mut At);
        linalg::matrix_transpose(&self.C, &mut Ct);
        linalg::matrix_identity(&mut I_p);

        // prediction step

        // x_prior = A * x
        linalg::matrix_times_vector(&self.A, &self.x, &mut x_prior);
		
        // P_prior = A * P * A^T + Q_w
        linalg::matrix_times_matrix(&self.P, &At, &mut PAt);
        linalg::matrix_times_matrix(&self.A, &PAt, &mut P_prior);
        linalg::matrix_plus_matrix(&mut P_prior, &self.Q_w);

        // correction step

        // K = P_prior * C^T * (C * P * C^T + Q_v)^-1
        linalg::matrix_times_matrix(&self.P, &Ct, &mut PCt);
        linalg::matrix_times_matrix(&self.C, &PCt, &mut CPCtQ);
        linalg::matrix_plus_matrix(&mut CPCtQ, &self.Q_v);
        linalg::matrix_invert(&mut CPCtQ); // todo: only for 2x2!
        linalg::matrix_times_matrix(&Ct, &CPCtQ, &mut CtCPCtQ);
        linalg::matrix_times_matrix(&P_prior, &CtCPCtQ, &mut self.K);

        // x = x_prior + K * (y - C * x_prior)
        linalg::matrix_times_vector(&self.C, &x_prior, &mut vec_l);
        linalg::vector_negate(&mut vec_l);
        linalg::vector_plus_vector(&mut vec_l, &y);
        linalg::matrix_times_vector(&self.K, &vec_l, &mut vec_p);
        linalg::vector_plus_vector(&mut self.x, &vec_p);

        // P = (I - K * C) * P_prior
        linalg::matrix_times_matrix(&self.K, &self.C, &mut KC);
        linalg::matrix_negate(&mut KC);
        linalg::matrix_plus_matrix(&mut KC, &I_p);
        linalg::matrix_times_matrix(&KC, &P_prior, &mut self.P);
    }
}

#[cfg(test)]
mod tests {
	use super::*;
	
	#[test]
	#[ignore]
	fn test_kalman() {
   	 	let x_init: [f32; 2] = [0., 0.];
   	 	let a = [[1., -0.001], [0., 1.]];
    	let c  = [[1., 0.]];
    	let q_w = [[0., 0.], [0., 0.]];
    	let q_v = [[0.]];
    	let p_init = [[0.0000001, 0.], [0., 0.0000001]];
    	let k_init = [[0.], [0.]];
    	let mut kalman = Kalman::new(x_init, a, c, q_w, q_v, p_init, k_init);

    	let y = [10.];
    	kalman.update_step(y);

		assert_eq!(kalman.x, [0., 0.]);
	}

} 
