use ngc_geometry::{
    profile::{
        LinearMotionProfile,
        LinearMotionSCurve,
        // SCurve, SCurveConstraints, SCurveStartConditions,
    },
    Vec3,
};
#[allow(unused_imports)]
use num_traits::Inv;
use poloto::prelude::*;

// use s_curve::*;

// use fmt::Write;
// use num_traits::Inv;
// use rand::Rng;
// use std::fmt;
// use tagger::new;

// fn main_1() {
//     let max_jerk: f32 = 3.;
//     let max_acceleration: f32 = 20.0;
//     let max_velocity: f32 = 30.;

//     let q0: f32 = 0.;
//     let q1: f32 = 50.;
//     let v0: f32 = 0.;
//     let v1: f32 = 0.;

//     #[allow(non_upper_case_globals)]
//     const range: u32 = 100;

//     let n_constraints = SCurveConstraints {
//         max_jerk,
//         max_acceleration,
//         max_velocity,
//     };
//     let n_start_conditions = SCurveStartConditions { q0, q1, v0, v1 };

//     let scurve = SCurve::new(&n_constraints, &n_start_conditions);

//     let mut std_output = Vec::<(f64, f64)>::new();
//     for i in 0..range {
//         let per = scurve.params.time_intervals.total_duration() / range as f32;
//         let x = i as f32 * per as f32;
//         let y = scurve.params.eval_velocity(x) as f64;

//         std_output.push((i as f64, y).into());
//     }

//     let title = format!("scurve with {} :", std_output.len(),);

//     let canvas = poloto::render::render_opt_builder()
//         // .with_precision(4)
//         // .with_dim([2000.0, 1000.0])
//         .with_tick_lines([true, true])
//         .build();

//     let plotter = poloto::simple_fmt!(
//         canvas,
//         plots!(
//             poloto::build::line("n", std_output), // poloto::build::line_fill_raw("", std_output.iter()),
//                                                   // poloto::build::markers([0.0, 1.0], [0.0, 1.0])
//         ),
//         title,
//         "x",
//         "y"
//     );
//     let w = create_svg_file("scurve.svg");
//     let _ = plotter.simple_theme_dark(w);
// }
mod draw;
fn main() {
    let enter_velocity: f32 = 48.0;
    let end_velocity: f32 = 0.0;
    let max_acceleration: f32 = 360000.;
    let max_velocity: f32 = 48.0;
    let steps: Vec3<i32> = Vec3::new(200, 700, 0);

    let mut linear = LinearMotionSCurve::new(
        100.,
        &steps,
        max_acceleration,
        max_velocity,
        enter_velocity,
        end_velocity,
    );

    let mut p_v = Vec::<(f64, f64)>::new();
    let mut d_v = Vec::<(f64, f64)>::new();
    while let Some((p, d)) = linear.next_profile() {
        p_v.push((p.0.into(), p.1.into()));
        d_v.push((p.0.into(), d.into()));

        // println!("{:?}, v:{}", prf.0, prf.1.inv());
    }

    // let first = all.first().unwrap();
    // let end = all.last().unwrap();
    // let title = format!(
    //     "Trapezoidal with {} delays:{:.3},{:.3}",
    //     all.len(),
    //     first,
    //     end
    // );
    let title = "";

    let canvas = poloto::render::render_opt_builder()
        .with_precision(4)
        // .with_dim([2000.0, 1000.0])
        .with_tick_lines([true, true])
        .build();

    let plotter = poloto::quick_fmt_opt!(
        canvas,
        title,
        "x",
        "y",
        poloto::build::line("velocity", d_v)
    );

    let w = draw::svg_file("scurve.svg");
    let _ = plotter.simple_theme_dark(w);
}
