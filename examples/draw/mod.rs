use std::path::Path;

use poloto::simple_theme::SimpleTheme;

pub fn svg_file(filename: &str) -> tagger::Adaptor<std::fs::File> {
    let base = Path::new("./target/assets/");
    if base.is_dir() == false {
        std::fs::create_dir_all(base).unwrap();
    }
    let base = base.to_path_buf();

    let path = base.join(Path::new(filename)).with_extension("svg");

    let file = std::fs::File::create(path).unwrap();
    tagger::upgrade_write(file)
}
