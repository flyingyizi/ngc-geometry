use ngc_geometry::{Arc, Vec3};

// use fmt::Write;
use poloto::prelude::*;
// use rand::Rng;
// use std::fmt;
mod draw;
fn main() {
    let start = Vec3::new(0., 0., 0.);
    let target = Vec3::new(5., 5., 0.);

    let radius = 5.0_f32;
    let turn_ccw = false;
    let abc = Arc::new_radius_mode(start, target, radius, turn_ccw);
    // let diff = Arc::check_center_mode(start, target, center_offset);
    // assert_eq!(diff, 0.0);

    //check output include target
    let all: Vec<_> = abc.collect();
    let title = format!("arc with {} segments:", all.len());

    let mut std_output = Vec::<(f64, f64)>::new();
    std_output.push((start.0.into(), start.1.into()));
    for i in all {
        std_output.push((i.0.into(), i.1.into()));
    }

    let canvas = poloto::render::render_opt_builder()
        //保持纵横比
        .preserve_aspect()
        .build();

    let plotter = poloto::quick_fmt_opt!(
        canvas,
        title,
        "x",
        "y",
        plots!(
            poloto::build::scatter("", std_output.iter()),
            // poloto::build::line_fill_raw("", std_output.iter()),
            // poloto::build::markers([0.0, 1.0], [0.0, 1.0])
        )
    );

    let w = draw::svg_file("arc_radius_mode.svg");
    let _ = plotter.simple_theme_dark(w);
}
