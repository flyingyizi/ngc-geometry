use ngc_geometry::{
    profile::{
        trapezoidal::{Conditions, Trapezoidal},
        LinearMotionProfile, LinearMotionTrapezoidal,
    },
    Vec3,
};

// use fmt::Write;
use num_traits::Inv;
use poloto::prelude::*;
// use rand::Rng;
// use std::fmt;
// use tagger::new;

mod draw;
// #[allow(dead_code)]
// fn main_1() {
//     let conditions = Conditions {
//         enter_velocity: 20.0,
//         end_velocity: 0.0,
//         target_accel: 21.5,
//     };

//     let max_velocity: f32 = 20.0;
//     let num_steps: u32 = 100;

//     // let init_velocity: f32 = 10.0;
//     // let end_velocity: f32 = 20.0;
//     // let max_velocity: f32 = 30.0;
//     // let target_accel: f32 = 1.5;
//     // let num_steps: u32 = 100;

//     // let init_velocity: f32 = 0.0;
//     // let end_velocity: f32 = 20.0;
//     // let max_velocity: f32 = 30.0;
//     // let target_accel: f32 = 1.5;
//     // let num_steps: u32 = 500;

//     // let init_velocity: f32 = 20.0;
//     // let end_velocity: f32 = 10.0;
//     // let max_velocity: f32 = 0.0;
//     // let target_accel: f32 = 1.5;
//     // let num_steps: u32 = 500;

//     let mut prof = Trapezoidal::new(Some(conditions), Some(max_velocity), num_steps);

//     let mut all: Vec<f64> = Vec::<f64>::new();
//     while let Some(delay) = prof.next_delay() {
//         all.push(delay.inv().into());
//     }

//     let first = all.first().unwrap();
//     let end = all.last().unwrap();
//     let title = format!(
//         "Trapezoidal with {} delays:{:.3},{:.3}",
//         all.len(),
//         first,
//         end
//     );

//     let std_output = (0..).zip(all.into_iter());

//     let canvas = poloto::render::render_opt_builder()
//         .with_precision(4)
//         // .with_dim([2000.0, 1000.0])
//         .with_tick_lines([true, true])
//         .build();

//     let plotter = poloto::quick_fmt_opt!(
//         canvas,
//         title,
//         "stemps",
//         "delays",
//         poloto::build::line("", std_output)
//     );

//     let w = create_svg_file("trapezoidal.svg");
//     let _ = plotter.simple_theme_dark(w);
// }

fn main() {
    let enter_velocity: f32 = 0.0;
    let end_velocity: f32 = 0.0;
    let target_accel: f32 = 21.5;

    let max_velocity: f32 = 20.0;
    // let num_steps: u32 = 100;

    let steps: Vec3<i32> = Vec3::new(500, 100, 0);
    let mut linear = LinearMotionTrapezoidal::new(
        &steps,
        target_accel,
        max_velocity,
        enter_velocity,
        end_velocity,
    );

    let mut p_v = Vec::<(f64, f64)>::new();
    let mut d_v = Vec::<(f64, f64)>::new();
    while let Some((p, d)) = linear.next_profile() {
        p_v.push((p.0.into(), p.1.into()));
        d_v.push((p.0.into(), d.into()));
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
        plots!(
            poloto::build::line("step", p_v),
            poloto::build::line("velocity", d_v) // poloto::build::line_fill_raw("", std_output.iter()),
                                                 // poloto::build::markers([0.0, 1.0], [0.0, 1.0])
        )
    );

    let w = draw::svg_file("trapezoidal.svg");
    let _ = plotter.simple_theme_dark(w);
}
