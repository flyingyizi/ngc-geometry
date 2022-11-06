use ngc_geometry::Vec2;
#[allow(unused_imports)]
use num_traits::Float;

/// after compensaton, the orig end pos will be to the new-end pos
fn compensation(start: &Vec2<f32>, end: &Vec2<f32>, radius: f32, side_is_left: bool) -> Vec2<f32> {
    let (cx, cy) = (start.0, start.1);
    let (px, py) = (end.0, end.1);

    let distance = (px - cx).hypot(py - cy);

    let theta = (radius / distance).acos();

    let alpha = if side_is_left {
        (cy - py).atan2(cx - px) - theta
    } else {
        (cy - py).atan2(cx - px) + theta
    };

    /* reset to end location */
    let cx = px + radius * alpha.cos();
    let cy = py + radius * alpha.sin();

    let end = Vec2::<f32>::new(cx, cy);
    end
}

//drawing for circle
fn circle(center: &Vec2<f32>, r: f32) -> Vec<(f64, f64)> {
    let mut line = Vec::<(f64, f64)>::new();

    let r = r as f64;
    let each = r / 100.;

    //  (x - x0)^2 +(y - y0)^2 =r^2
    // sqrt(r^2 - (x - x0)^2) +y0
    let (_x0, _y0) = (center.0 as f64, center.1 as f64);

    let begin = _x0 - r;
    for i in 0..200 {
        let x = (begin + each * i as f64) as f64;
        let t = (r * r - (x - _x0).powi(2)).sqrt() as f64;
        line.push((x, t + _y0));
        line.push((x, _y0 - t));
    }
    line
}

// use fmt::Write;
use poloto::prelude::*;
// use rand::Rng;
// use std::fmt;
mod draw;

fn main() {
    let start = Vec2::<f32>::new(2., 2.);
    let end = Vec2::<f32>::new(16., 16.);

    let radius = 1.0_f32;
    let new_end = compensation(&start, &end, radius, false);
    println!("old-end:{:?}, new-ned{:?}", end, new_end);
    let mut line1 = Vec::<(f64, f64)>::new();
    line1.push((start.0.into(), start.1.into()));
    line1.push((end.0.into(), end.1.into()));

    let mut line2 = Vec::<(f64, f64)>::new();
    line2.push((start.0.into(), start.1.into()));
    line2.push((new_end.0.into(), new_end.1.into()));

    let circle_d = circle(&end, radius);

    let canvas = poloto::render::render_opt_builder()
        //保持纵横比
        .with_tick_lines([true, true])
        .preserve_aspect()
        .build();

    let plotter = poloto::quick_fmt_opt!(
        canvas,
        "title",
        "x",
        "y",
        poloto::build::scatter("", circle_d.iter()),
        poloto::build::line("", line1.iter()),
        poloto::build::line("", line2.iter()) // poloto::build::line(name, it)
    );

    let w = draw::svg_file("compensation.svg");
    let _ = plotter.simple_theme_dark(w);
}
