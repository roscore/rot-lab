//! Tiny DOM helpers over web-sys.

use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, Event, EventTarget, HtmlInputElement, HtmlSelectElement, HtmlTextAreaElement, Window};

pub fn win() -> Window {
    web_sys::window().expect("window")
}

pub fn doc() -> Document {
    win().document().expect("document")
}

pub fn el(id: &str) -> Element {
    doc().get_element_by_id(id).unwrap_or_else(|| panic!("missing element #{id}"))
}

/// Value of an input / select / textarea.
pub fn get(id: &str) -> String {
    let e = el(id);
    if let Some(i) = e.dyn_ref::<HtmlInputElement>() {
        i.value()
    } else if let Some(s) = e.dyn_ref::<HtmlSelectElement>() {
        s.value()
    } else if let Some(t) = e.dyn_ref::<HtmlTextAreaElement>() {
        t.value()
    } else {
        String::new()
    }
}

pub fn set(id: &str, v: &str) {
    let e = el(id);
    if let Some(i) = e.dyn_ref::<HtmlInputElement>() {
        i.set_value(v);
    } else if let Some(s) = e.dyn_ref::<HtmlSelectElement>() {
        s.set_value(v);
    } else if let Some(t) = e.dyn_ref::<HtmlTextAreaElement>() {
        t.set_value(v);
    }
}

pub fn checked(id: &str) -> bool {
    el(id).dyn_ref::<HtmlInputElement>().map(|i| i.checked()).unwrap_or(false)
}

pub fn text(id: &str, s: &str) {
    el(id).set_text_content(Some(s));
}

pub fn show(id: &str, on: bool) {
    show_el(&el(id), on);
}

pub fn show_el(e: &Element, on: bool) {
    if on {
        let _ = e.remove_attribute("hidden");
    } else {
        let _ = e.set_attribute("hidden", "");
    }
}

pub fn set_class(e: &Element, class: &str, on: bool) {
    let _ = e.class_list().toggle_with_force(class, on);
}

/// Show a message paragraph with a kind: "warn", "err", "ok" or "" to hide.
pub fn msg(id: &str, kind: &str, s: &str) {
    let e = el(id);
    if kind.is_empty() || s.is_empty() {
        show_el(&e, false);
        return;
    }
    e.set_class_name(&format!("msg {kind}"));
    e.set_text_content(Some(s));
    show_el(&e, true);
}

pub fn query_all(sel: &str) -> Vec<Element> {
    let list = doc().query_selector_all(sel).expect("selector");
    (0..list.length()).filter_map(|i| list.item(i)).filter_map(|n| n.dyn_into::<Element>().ok()).collect()
}

pub fn on(target: &EventTarget, ev: &str, f: impl FnMut(Event) + 'static) {
    let c = Closure::<dyn FnMut(Event)>::new(f);
    target.add_event_listener_with_callback(ev, c.as_ref().unchecked_ref()).expect("listener");
    c.forget();
}

pub fn on_id(id: &str, ev: &str, f: impl FnMut(Event) + 'static) {
    on(el(id).as_ref(), ev, f);
}

/// Copy text to the clipboard (async, best effort).
pub fn copy(s: &str) {
    let _ = win().navigator().clipboard().write_text(s);
}

pub fn storage_get(k: &str) -> Option<String> {
    win().local_storage().ok().flatten().and_then(|s| s.get_item(k).ok().flatten())
}

pub fn storage_set(k: &str, v: &str) {
    if let Some(s) = win().local_storage().ok().flatten() {
        let _ = s.set_item(k, v);
    }
}

pub fn dark_mode() -> bool {
    win().match_media("(prefers-color-scheme: dark)").ok().flatten().map(|m| m.matches()).unwrap_or(false)
}
