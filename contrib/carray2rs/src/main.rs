use std::env;
use std::fs;
use std::io::{self, Read, Write};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BaseTy { U8, U32 }

#[derive(Debug)]
struct CArray {
    c_name: String,
    rust_name: String,
    ty: BaseTy,
    values: Vec<u64>,
}

fn main() -> io::Result<()> {
    let mut args = env::args().skip(1);
    let mut alias_patch = false;
    let mut input_path: Option<String> = None;
    let mut out_path: Option<String> = None;

    while let Some(a) = args.next() {
        match a.as_str() {
            "--alias-patch" => alias_patch = true,
            "--out" => out_path = args.next(),
            _ => {
                if input_path.is_none() { input_path = Some(a); } else { eprintln!("unexpected arg: {}", a); }
            }
        }
    }

    let mut input = String::new();
    match input_path.as_deref() {
        Some("-") | None => { io::stdin().read_to_string(&mut input)?; }
        Some(p) => { input = fs::read_to_string(p)?; }
    }

    let stripped = strip_comments(&input);
    let arrays = parse_c_arrays(&stripped, alias_patch);

    if arrays.is_empty() {
        eprintln!("No arrays found in input.");
        std::process::exit(1);
    }

    let mut out = String::new();
    out.push_str("#![allow(dead_code)]\n\n");
    for a in arrays {
        let ty = match a.ty { BaseTy::U8 => "u8", BaseTy::U32 => "u32" };
        let len = a.values.len();
        out.push_str("#[rustfmt::skip]\n");
        out.push_str(&format!("pub const {}: [{}; {}] = [\n", a.rust_name, ty, len));
        // pretty-print 8 per line
        let mut line = String::new();
        for (i, v) in a.values.iter().enumerate() {
            let s = if matches!(a.ty, BaseTy::U8) { format!("0x{:02X}", v) } else { format!("0x{:08X}", v) };
            if !line.is_empty() { line.push_str(", "); }
            line.push_str(&s);
            if (i + 1) % 8 == 0 { out.push_str("    "); out.push_str(&line); out.push_str(",\n"); line.clear(); }
        }
        if !line.is_empty() { out.push_str("    "); out.push_str(&line); out.push_str(",\n"); }
        out.push_str("];\n\n");
    }

    match out_path {
        Some(p) => fs::write(p, out),
        None => {
            let mut stdout = io::stdout();
            stdout.write_all(out.as_bytes())
        }
    }
}

fn strip_comments(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '/' {
            match chars.peek().copied() {
                Some('/') => { // line comment
                    chars.next();
                    while let Some(ch) = chars.next() { if ch == '\n' { out.push('\n'); break; } }
                    continue;
                }
                Some('*') => { // block comment
                    chars.next();
                    let mut prev = '\0';
                    while let Some(ch) = chars.next() {
                        if prev == '*' && ch == '/' { break; }
                        prev = ch;
                    }
                    continue;
                }
                _ => {}
            }
        }
        out.push(c);
    }
    out
}

fn parse_c_arrays(s: &str, alias_patch: bool) -> Vec<CArray> {
    let mut out = Vec::new();
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        // find '=' as a heuristic for declarations
        if bytes[i] == b'=' {
            // scan backwards to start of decl (from last ';' or line start)
            let decl_start = s[..i].rfind(';').map(|p| p + 1).unwrap_or(0);
            let decl = s[decl_start..i].trim();
            // Find the '{...}' after '='
            let mut j = i + 1;
            while j < bytes.len() && bytes[j].is_ascii_whitespace() { j += 1; }
            if j >= bytes.len() || bytes[j] != b'{' { i += 1; continue; }
            let (end_brace, inner) = match extract_brace_block(&s[j..]) { Some(v) => v, None => { i += 1; continue; } };

            if let Some(arr) = parse_decl_and_values(decl, inner, alias_patch) {
                out.push(arr);
            }

            i = j + end_brace; // continue after the closing brace
            continue;
        }
        i += 1;
    }
    out
}

fn extract_brace_block(s: &str) -> Option<(usize, &str)> {
    assert!(s.as_bytes().first().copied() == Some(b'{'));
    let mut depth = 0usize;
    for (idx, ch) in s.char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 { return Some((idx + 1, &s[1..idx])); }
            }
            _ => {}
        }
    }
    None
}

fn parse_decl_and_values(decl: &str, body: &str, alias_patch: bool) -> Option<CArray> {
    // Expect: [qualifiers] <type tokens> <name> [ ... ]
    let mut name = String::new();
    let mut ty = BaseTy::U32; // default

    // Find name: identifier immediately before '['
    let lb = decl.rfind('[')?;
    let before = decl[..lb].trim_end();
    // scan backwards to the identifier
    let mut id_end = before.len();
    let mut id_start = id_end;
    for (pos, ch) in before.char_indices().rev() {
        if ch == ' ' || ch == '\t' || ch == '*' { id_start = pos + 1; break; }
        id_start = pos;
        if pos == 0 { break; }
    }
    name.push_str(&before[id_start..id_end]);
    if name.is_empty() { return None; }

    // Determine base type from the prefix
    let prefix = before[..id_start].trim();
    let p = collapse_ws(prefix);
    let p_lower = p.to_lowercase();
    if p_lower.contains("uint8_t") || p_lower.contains("unsignedchar") { ty = BaseTy::U8; }
    else if p_lower.contains("uint32_t") || p_lower.contains("unsignedint") { ty = BaseTy::U32; }
    // else remain default u32

    // Values
    let values = parse_values(body)?;

    // Rust name
    let rust_name = if alias_patch {
        match name.as_str() {
            "g_lcpu_patch_list" => "PATCH_RECORD_U32".to_string(),
            "g_lcpu_patch_bin" => "PATCH_CODE_U32".to_string(),
            _ => to_screaming_snake(&name, Some(ty)),
        }
    } else {
        to_screaming_snake(&name, Some(ty))
    };

    Some(CArray { c_name: name, rust_name, ty, values })
}

fn collapse_ws(s: &str) -> String {
    let mut out = String::new();
    let mut last_space = false;
    for ch in s.chars() {
        if ch.is_whitespace() { if !last_space { out.push(' '); last_space = true; } }
        else { out.push(ch); last_space = false; }
    }
    out.replace(' ', "")
}

fn to_screaming_snake(name: &str, ty: Option<BaseTy>) -> String {
    let mut out = String::new();
    let mut prev_is_lower = false;
    for ch in name.chars() {
        if ch.is_ascii_alphanumeric() {
            if ch.is_ascii_uppercase() {
                if prev_is_lower { out.push('_'); }
                out.push(ch);
                prev_is_lower = false;
            } else { // lowercase or digit
                out.push(ch.to_ascii_uppercase());
                prev_is_lower = ch.is_ascii_lowercase();
            }
        } else {
            if !out.ends_with('_') { out.push('_'); }
            prev_is_lower = false;
        }
    }
    match ty {
        Some(BaseTy::U8) => format!("{}_U8", out.trim_matches('_')),
        Some(BaseTy::U32) => format!("{}_U32", out.trim_matches('_')),
        None => out.trim_matches('_').to_string(),
    }
}

fn parse_values(body: &str) -> Option<Vec<u64>> {
    let mut vals = Vec::new();
    for raw in body.split(',') {
        let t = raw.trim();
        if t.is_empty() { continue; }
        let t = t.trim_matches(|c: char| c == '{' || c == '}' || c.is_whitespace());
        if t.is_empty() { continue; }
        // strip suffixes U/L
        let mut core = t.trim_end_matches(|c: char| c == 'u' || c == 'U' || c == 'l' || c == 'L');
        // remove casts like (uint32_t)
        if let Some(idx) = core.rfind(')') { if let Some(st) = core.find('(') { if st < idx { core = &core[idx+1..]; } } }
        let val = if let Some(h) = core.strip_prefix("0x").or_else(|| core.strip_prefix("0X")) {
            u64::from_str_radix(h.trim(), 16).ok()?
        } else {
            core.trim().parse::<u64>().ok()?
        };
        vals.push(val);
    }
    Some(vals)
}

