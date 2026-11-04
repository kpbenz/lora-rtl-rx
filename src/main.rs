//! LoRa receiver GUI.

mod gui;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("LoRa RTL-SDR Receiver")
            .with_inner_size([1320.0, 860.0])
            .with_min_inner_size([980.0, 620.0]),
        ..Default::default()
    };
    eframe::run_native(
        "lora-rtl-rx",
        options,
        Box::new(|creation| Ok(Box::new(gui::App::new(creation)))),
    )
}
