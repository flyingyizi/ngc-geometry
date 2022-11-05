use ngc_geometry::{line::Line3D, vecx::Point3};

// use fmt::Write;
use poloto::prelude::*;
// use rand::Rng;
// use std::fmt;
mod draw;
fn main() {
    let start = Point3::new(2, 2, 0);
    let end = Point3::new(16, 6, 0);
    let bi = Line3D::new(start, end);

    let all: Vec<_> = bi.collect();
    let title = format!("line with {} len:", all.len());

    let mut std_output = Vec::<(f64, f64)>::new();
    for i in all {
        std_output.push((i.0.into(), i.1.into()));
    }

    let canvas = poloto::render::render_opt_builder()
        //保持纵横比
        .with_tick_lines([true, true])
        .preserve_aspect()
        .build();

    let plotter = poloto::quick_fmt_opt!(
        canvas,
        title,
        "x",
        "y",
        poloto::build::scatter("", std_output.iter())
    );

    let w = draw::svg_file("line3d.svg");
    let _ = plotter.simple_theme_dark(w);
}
