// Parsing and writing ~/.config/qy8boot/units. Pure Foundation, no AppKit,
// so it can be exercised from a small command-line driver as well as from
// ConfigUI.swift. Keep this file's format in lockstep with qy8boot and
// _qy8boot (tools/qy8/qy8boot, tools/qy8/_qy8boot): five whitespace-separated
// fields, "-" for an empty extra, "#" comments, blank lines allowed anywhere.
import Foundation

struct PresetLine {
    var raw: String              // original text, used verbatim for comments/blank lines
    var isPreset: Bool
    var unit: String = ""
    var variant: String = ""
    var card: String = ""
    var nand: String = ""
    var extra: String = ""       // "" means none; written as "-"
}

func parseUnitsFile(_ path: String) -> [PresetLine] {
    guard let text = try? String(contentsOfFile: path, encoding: .utf8) else { return [] }
    var raws = text.components(separatedBy: "\n")
    if raws.last == "" { raws.removeLast() }
    var out: [PresetLine] = []
    for s in raws {
        let t = s.trimmingCharacters(in: .whitespaces)
        if t.isEmpty || t.hasPrefix("#") {
            out.append(PresetLine(raw: s, isPreset: false))
            continue
        }
        let f = t.split(separator: " ", omittingEmptySubsequences: true).map(String.init)
        guard f.count >= 4 else {
            out.append(PresetLine(raw: s, isPreset: false))
            continue
        }
        let extraRaw = f.count > 4 ? f[4...].joined(separator: " ") : "-"
        out.append(PresetLine(raw: s, isPreset: true, unit: f[0], variant: f[1],
                               card: f[2], nand: f[3], extra: extraRaw == "-" ? "" : extraRaw))
    }
    return out
}

func presetLineText(_ p: PresetLine) -> String {
    let extra = p.extra.isEmpty ? "-" : p.extra
    return "\(p.unit) \(p.variant) \(p.card) \(p.nand) \(extra)"
}

func pathHasSpace(_ p: String) -> Bool { p.contains(" ") }

// Creates the file and its folder with just the header if nothing is there yet.
func ensureUnitsFile(_ path: String) {
    guard !FileManager.default.fileExists(atPath: path) else { return }
    let dir = (path as NSString).deletingLastPathComponent
    if !dir.isEmpty {
        try? FileManager.default.createDirectory(atPath: dir, withIntermediateDirectories: true)
    }
    let header = "# ~/.config/qy8boot/units - qy8boot's unit presets.\n"
        + "# unit variant card nand extra-flags ('-' = none). First variant per unit is the default.\n"
    try? header.write(toFile: path, atomically: true, encoding: .utf8)
}

// Temp file plus rename, same directory, so a reader never sees a half
// written file; the previous version is kept beside it as units.bak.
func writeUnitsFile(_ lines: [PresetLine], to path: String) throws {
    let dir = (path as NSString).deletingLastPathComponent
    if !dir.isEmpty {
        try FileManager.default.createDirectory(atPath: dir, withIntermediateDirectories: true)
    }
    let body = lines.map { $0.isPreset ? presetLineText($0) : $0.raw }.joined(separator: "\n") + "\n"

    let fm = FileManager.default
    if fm.fileExists(atPath: path) {
        let backup = dir.isEmpty ? "units.bak" : dir + "/units.bak"
        try? fm.removeItem(atPath: backup)
        try fm.copyItem(atPath: path, toPath: backup)
    }

    let tmp = path + ".tmp-\(ProcessInfo.processInfo.globallyUniqueString)"
    try body.write(toFile: tmp, atomically: false, encoding: .utf8)
    if fm.fileExists(atPath: path) {
        _ = try fm.replaceItemAt(URL(fileURLWithPath: path), withItemAt: URL(fileURLWithPath: tmp))
    } else {
        try fm.moveItem(atPath: tmp, toPath: path)
    }
}

// Keeps a new preset beside any existing lines for the same unit, so
// --list's one-line-per-unit grouping still holds; a unit seen for the
// first time goes at the end.
func insertGrouped(_ lines: inout [PresetLine], _ new: PresetLine) {
    if let lastIdx = lines.lastIndex(where: { $0.isPreset && $0.unit == new.unit }) {
        lines.insert(new, at: lastIdx + 1)
    } else {
        lines.append(new)
    }
}

// qy8boot.sh's own short flags (-m/-b) live on the popups, not in free
// text; anything else is a literal qemu flag and stays in the Extra field.
func splitPresetExtra(_ extra: String) -> (model: String?, board: String?, rest: String) {
    let tokens = extra.split(separator: " ").map(String.init)
    var model: String?, board: String?
    var rest: [String] = []
    var i = 0
    while i < tokens.count {
        switch tokens[i] {
        case "-m" where i + 1 < tokens.count: model = tokens[i + 1]; i += 2
        case "-b" where i + 1 < tokens.count: board = tokens[i + 1]; i += 2
        default: rest.append(tokens[i]); i += 1
        }
    }
    return (model, board, rest.joined(separator: " "))
}

func joinPresetExtra(model: String, board: String, rest: String) -> String {
    var tokens: [String] = []
    if model != "Auto" { tokens.append("-m"); tokens.append(model) }
    if board != "Auto" { tokens.append("-b"); tokens.append(board) }
    let restTrim = rest.trimmingCharacters(in: .whitespaces)
    if !restTrim.isEmpty { tokens.append(contentsOf: restTrim.split(separator: " ").map(String.init)) }
    return tokens.joined(separator: " ")
}
