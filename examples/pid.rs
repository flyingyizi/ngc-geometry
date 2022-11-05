use ngc_geometry::pid::PID;
use poloto::build::bar::gen_bar;
use rand::{
    distributions::{Distribution, Uniform},
    Rng,
};
mod draw;
use poloto::prelude::*;

fn pid_output(kp: f32, ki: f32, kd: f32) -> Vec<(f64, f64)> {
    let setpoint: f32 = 100.;

    // let kp: f32 = 0.005;
    // let ki: f32 = 0.; // 0.12;
    // let kd: f32 = 0.; // 0.05;
    // let sampletime: f32 = 0.016; //16ms

    let mut rng = rand::thread_rng();
    let range = Uniform::new(0.0_f32, 1.0_f32);

    //由于setpoit是100，所以system_modal outupt会趋向20
    let mut system_modal = |a: f32| -> f32 {
        let s = range.sample(&mut rng);

        5.0 * a + s
    };

    // Prepare PID controller for operation
    let mut pid = PID::new(setpoint, kp, ki, kd);

    // int time_length = 1600; //
    let K = 100; //
                 // auto t = linspace(0, time_length, K);

    // 记录被控对象输入
    let mut u: f32;
    // # k=0时刻被控对象输入值
    let u_0 = 0.0_f32; // random 此时未经过pid控制器，随机选择一个值
                       //保存被控对象输出
    let mut real_output = Vec::<f32>::new();
    real_output.push(system_modal(u_0));

    let mut p_v = Vec::<(f64, f64)>::new();
    for k in 1..K {
        let out = pid.PIDCompute(real_output[k - 1]);
        // println!("---{}", out); //这个out会趋向20
        let out = system_modal(out);
        real_output.push(out); //# 计算下一时刻(k)被控对象新的输出值
    }

    pid.PIDSetSetpoint(2. * setpoint);
    for k in K..2 * K {
        let out = pid.PIDCompute(real_output[k - 1]);
        // println!("---{}", out); //这个out会趋向20
        let out = system_modal(out);
        real_output.push(out); //# 计算下一时刻(k)被控对象新的输出值
    }

    //////////////////////////////////////
    let mut i = 0.;
    for it in real_output {
        p_v.push((i, it as f64));
        i += 1.;
    }

    p_v
}

fn main() {
    let kp: f32 = 0.005;
    let ki: f32 = 0.22;
    let kd: f32 = 0.0022;
    let p_v0 = pid_output(kp, ki, 0.0012);
    let p_v1 = pid_output(kp, ki, kd);
    let p_v2 = pid_output(kp, ki, 0.032);

    let canvas = poloto::render::render_opt_builder()
        .with_precision(4)
        // .with_dim([2000.0, 1000.0])
        .with_tick_lines([true, true])
        .build();

    let plotter = poloto::quick_fmt_opt!(
        canvas,
        "title",
        "x",
        "y",
        poloto::build::line("0", p_v0.iter()),
        poloto::build::line("1", p_v1.iter()) // poloto::build::line("2", p_v2.iter())
    );

    let w = draw::svg_file("pid.svg");
    let _ = plotter.simple_theme_dark(w);
}
