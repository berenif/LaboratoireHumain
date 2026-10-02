#[cfg(not(target_arch = "wasm32"))]
fn main() {}

#[cfg(target_arch = "wasm32")]
mod browser {
    use leptos::prelude::*;
    use lh_renderer::BrowserRenderer;
    use serde::Deserialize;
    use wasm_bindgen::prelude::*;
    use web_sys::{CustomEvent, CustomEventInit};

    #[wasm_bindgen(module = "/driver.js")]
    extern "C" {
        fn start_bridge();
        fn stop_bridge();
    }

    #[wasm_bindgen]
    pub async fn make_renderer(
        canvas: web_sys::HtmlCanvasElement,
        backend: String,
    ) -> Result<BrowserRenderer, JsValue> {
        BrowserRenderer::create(canvas, &backend).await
    }
    #[wasm_bindgen]
    pub fn linear_memory_bytes() -> usize {
        core::arch::wasm32::memory_size(0) * 65536
    }
    fn emit(value: serde_json::Value) {
        let init = CustomEventInit::new();
        init.set_detail(&JsValue::from_str(&value.to_string()));
        if let (Some(window), Ok(event)) = (
            web_sys::window(),
            CustomEvent::new_with_event_init_dict("lh-action", &init),
        ) {
            let _ = window.dispatch_event(&event);
        }
    }
    #[derive(Clone, Default, Deserialize)]
    #[serde(default, rename_all = "camelCase")]
    struct UiStatus {
        ready: bool,
        can_reset: bool,
        halted: bool,
        heading: f64,
        paused: bool,
        mode: String,
        station: String,
        difficulty: String,
        backend: String,
        requested_renderer: String,
        quality: String,
        selected: String,
        tick: u64,
        generation: u64,
        time: f64,
        fps: f64,
        realtime_ratio: f64,
        steps: u32,
        strikes: u32,
        can_strike: bool,
        striker_phase: String,
        motion: String,
        falls: u32,
        contacts: u32,
        floor_enabled: bool,
        desired_floor_enabled: bool,
        error: String,
    }
    #[component]
    fn App() -> impl IntoView {
        let status = RwSignal::new(UiStatus {
            mode: "protocol".into(),
            quality: "auto".into(),
            floor_enabled: true,
            desired_floor_enabled: true,
            ..Default::default()
        });
        let listener = Closure::<dyn FnMut(web_sys::Event)>::new(move |event: web_sys::Event| {
            if let Some(event) = event.dyn_ref::<CustomEvent>()
                && let Some(text) = event.detail().as_string()
                && let Ok(value) = serde_json::from_str(&text)
            {
                status.set(value);
            }
        });
        let window = web_sys::window().expect("browser window");
        window
            .add_event_listener_with_callback("lh-status", listener.as_ref().unchecked_ref())
            .expect("status listener");
        let listener = StoredValue::new_local((window, listener));
        on_cleanup(move || {
            listener.with_value(|(window, callback)| {
                let _ = window.remove_event_listener_with_callback(
                    "lh-status",
                    callback.as_ref().unchecked_ref(),
                );
            });
            stop_bridge();
        });
        view! {
            <header class="topbar">
                <a class="brand" href="./"><span class="brand-mark">"◎"</span><span>"LABORATOIRE"<b>"HUMAIN"</b></span></a>
                <span class="preview-tag">"Rust preview · acceptance in progress"</span>
            </header>
            <main class="workspace">
                <aside class="panel">
                    <p class="eyebrow">"THE BALANCE EXPERIMENT"</p>
                    <h1>"A body in"<br/>"balance."</h1>
                    <p class="intro">"Explore how a physical body responds to a gentle pull. Drag a surface, release it, and watch its response."</p>
                    <div class="mode-tabs" role="group" aria-label="Experiment mode">
                        <button class:active=move||status.get().mode=="protocol" disabled=move||!status.get().ready on:click=move|_|emit(serde_json::json!({"type":"mode","value":"protocol"}))>"Impact protocol"</button>
                        <button class:active=move||status.get().mode=="playground" disabled=move||!status.get().ready on:click=move|_|emit(serde_json::json!({"type":"mode","value":"playground"}))>"Balance playground"</button>
                    </div>
                    <div class="controls">
                        <div class="control-heading"><h2>"Trial controls"</h2><span class="key">"SPACE"</span></div>
                        <div class="button-row">
                            <button id="pause" class="primary" disabled=move||!status.get().ready on:click=move|_|emit(serde_json::json!({"type":"pause"}))>{move||if status.get().paused {"Resume"}else{"Pause"}}</button>
                            <button id="reset" disabled=move||!status.get().can_reset on:click=move|_|emit(serde_json::json!({"type":"reset"}))>"Reset"</button>
                        </div>
                        <label class="toggle"><input id="floor" type="checkbox" disabled=move||!status.get().ready prop:checked=move||status.get().desired_floor_enabled on:change=move|e|emit(serde_json::json!({"type":"floor","value":event_target_checked(&e)}))/><span>"Ground enabled"</span></label>
                        <label>"Starting heading"<select id="heading" disabled=move||!status.get().ready prop:value=move||status.get().heading.to_string() on:change=move|e|emit(serde_json::json!({"type":"heading","value":event_target_value(&e)}))><option value="0">"Front · 0°"</option><option value="1.0471975512">"Turn · +60°"</option><option value="-0.7853981634">"Turn · −45°"</option></select></label>
                        <label>"Select a region"<select id="region" prop:value=move||status.get().selected on:change=move|e|emit(serde_json::json!({"type":"region","value":event_target_value(&e)}))>
                            <option value="">"All surfaces"</option><option value="head">"Head"</option><option value="torso">"Torso"</option><option value="pelvis">"Pelvis"</option>
                            <option value="leftHand">"Left hand"</option><option value="rightHand">"Right hand"</option><option value="leftFoot">"Left foot"</option><option value="rightFoot">"Right foot"</option>
                        </select></label>
                        <Show when=move||status.get().mode=="playground">
                            <label>"Station"<select id="station" disabled=move||!status.get().ready prop:value=move||status.get().station on:change=move|e|emit(serde_json::json!({"type":"station","value":event_target_value(&e)}))>
                                <option value="flat">"Base camp"</option><option value="slope">"The incline"</option><option value="rubble">"Broken ground"</option><option value="beam">"Knife edge"</option><option value="stones">"Islands"</option><option value="wobble">"Sea legs"</option><option value="hurdles">"Trip wire"</option>
                            </select></label>
                            <label>"Difficulty"<select id="difficulty" disabled=move||!status.get().ready prop:value=move||status.get().difficulty on:change=move|e|emit(serde_json::json!({"type":"difficulty","value":event_target_value(&e)}))>
                                <option value="gentle">"Gentle"</option><option value="challenging">"Challenging"</option><option value="extreme">"Extreme"</option>
                            </select></label>
                            <p class="migration-note">"Changing stations or difficulty starts a fresh trial."</p>
                        </Show>
                        <Show when=move||status.get().mode=="protocol">
                            <div class="control-heading"><h2>"Impact protocol"</h2><kbd>"P"</kbd></div>
                            <button id="strike" class="primary" disabled=move||!status.get().can_strike on:click=move|_|emit(serde_json::json!({"type":"strike"}))>"Strike"</button>
                            <p aria-live="polite">{move||format!("{} · {} measured impacts",status.get().striker_phase,status.get().strikes)}</p>
                        </Show>
                    </div>
                    <div class="view-controls">
                        <div class="control-heading"><h2>"View"</h2><button class="text-button" id="camera-focus" on:click=move|_|emit(serde_json::json!({"type":"focus"}))>"Focus body"</button><button class="text-button" id="camera-reset" on:click=move|_|emit(serde_json::json!({"type":"camera"}))>"Reset camera"</button></div>
                        <div class="select-row"><label>"Renderer"<select id="renderer" prop:value=move||status.get().requested_renderer on:change=move|e|emit(serde_json::json!({"type":"renderer","value":event_target_value(&e)}))><option value="auto">"Auto"</option><option value="webgpu">"WebGPU"</option><option value="webgl2">"WebGL2"</option></select></label>
                        <label>"Quality"<select id="quality" prop:value=move||status.get().quality on:change=move|e|emit(serde_json::json!({"type":"quality","value":event_target_value(&e)}))><option value="auto">"Auto"</option><option value="low">"Low"</option><option value="high">"High"</option></select></label></div>
                    </div>
                    <div class="body-spec"><span><b>"25"</b>" segments"</span><span><b>"72.2"</b>" kg"</span><span><b>"1.84"</b>" m"</span></div>
                </aside>
                <section class="scene" aria-label="Physical experiment" data-tick=move||status.get().tick.to_string() data-generation=move||status.get().generation.to_string()>
                    <div class="scene-top"><span class="live-dot"></span><span>{move||if status.get().halted {"Trial stopped".to_string()}else if !status.get().ready {"Starting experiment".to_string()}else if status.get().paused {"Trial paused".to_string()}else{"Trial running".to_string()}}</span><span class="backend">{move||status.get().backend}</span></div>
                    <div id="viewport"></div>
                    <div class="scene-help"><span>"Drag body · pull"</span><span>"Drag empty space · orbit"</span><span>"Scroll · zoom"</span></div>
                    <div class="error-banner" hidden=move||status.get().error.is_empty()><p>{move||status.get().error}</p><button id="graphics-retry" on:click=move|_|emit(serde_json::json!({"type":"retry"}))>"Retry graphics"</button></div>
                </section>
                <section class="metrics" aria-label="Live trial measurements">
                    <div><span>"BODY STATE"</span><strong id="motion">{move||status.get().motion}</strong><small>{move||format!("{} falls",status.get().falls)}</small></div>
                    <div><span>"PHYSICS TIME"</span><strong>{move||format!("{:.2}",status.get().time)}<small>" s"</small></strong></div>
                    <div><span>"MEASURED SUPPORT"</span><strong>{move||status.get().contacts}<small>" segments"</small></strong></div>
                    <div><span>"COMPLETED STEPS"</span><strong>{move||status.get().steps}</strong></div>
                    <div><span>"PRESENTATION"</span><strong>{move||format!("{:.0}",status.get().fps)}<small>" fps"</small></strong></div>
                    <div><span>"SIMULATION / WALL"</span><strong>{move||format!("{:.2}×",status.get().realtime_ratio)}</strong></div>
                </section>
            </main>
            <footer>"Physical acceptance remains open. An upright picture alone does not establish balance or recovery."</footer>
        }
    }
    pub fn run() {
        console_error_panic_hook::set_once();
        leptos::mount::mount_to_body(App);
        start_bridge();
    }
}
#[cfg(target_arch = "wasm32")]
fn main() {
    browser::run();
}
