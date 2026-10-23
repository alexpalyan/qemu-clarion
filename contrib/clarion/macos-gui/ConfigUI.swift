// QY8 Emulator config window. Compiled standalone with swiftc, alongside
// PresetStore.swift, into Contents/MacOS/QY8ConfigUI; launcher execs it
// after setting QY8_RES/QY8_LOG_DIR. AppKit only, no nib: built
// programmatically so package-app.sh can swiftc it without Xcode.
import AppKit
import UniformTypeIdentifiers

let defaults = UserDefaults.standard
let res = ProcessInfo.processInfo.environment["QY8_RES"] ?? Bundle.main.bundlePath + "/../Resources"
let logDir = ProcessInfo.processInfo.environment["QY8_LOG_DIR"]
    ?? (NSHomeDirectory() + "/Library/Logs/QY8 Emulator")
let bootSh = res + "/boot.sh"
let quickstart = res + "/QUICKSTART.txt"
// QY8_UNITS overrides the presets file, for testing against a copy instead
// of the real ~/.config/qy8boot/units.
let presetsPath = ProcessInfo.processInfo.environment["QY8_UNITS"]
    ?? (NSHomeDirectory() + "/.config/qy8boot/units")

struct Preset { let label: String; let card: String; let nand: String; let extra: String }

func loadPresets() -> [Preset] {
    parseUnitsFile(presetsPath).filter { $0.isPreset }.map {
        Preset(label: "\($0.unit) \($0.variant)", card: $0.card, nand: $0.nand, extra: $0.extra)
    }
}

func runCapture(_ exe: String, _ args: [String]) -> String {
    let p = Process()
    p.executableURL = URL(fileURLWithPath: exe)
    p.arguments = args
    let pipe = Pipe()
    p.standardOutput = pipe
    p.standardError = pipe
    do { try p.run() } catch { return "" }
    let data = pipe.fileHandleForReading.readDataToEndOfFile()
    p.waitUntilExit()
    return String(data: data, encoding: .utf8) ?? ""
}

final class ConfigWindow: NSObject, NSApplicationDelegate, NSWindowDelegate {
    var window: NSWindow!
    var cardField: NSTextField!
    var nandField: NSTextField!
    var unitLabel: NSTextField!
    var statusLabel: NSTextField!
    var presetPopup: NSPopUpButton!
    var keepBox: NSButton!
    var verboseBox: NSButton!
    var boardPopup: NSPopUpButton!
    var modelPopup: NSPopUpButton!
    var extraField: NSTextField!
    var runButton: NSButton!
    var restartButton: NSButton!
    var copyLogButton: NSButton!
    var optionsToggle: NSButton!
    var optionsView: NSView!
    var logTextView: NSTextView!
    var presets: [Preset] = []
    var runProcess: Process?
    var qemuPid: Int32?
    var tailTimer: Timer?
    var consoleOffset: UInt64 = 0
    var qemuLogOffset: UInt64 = 0
    var isRunning = false
    var detectedUnitCode: String?   // NAND_QY, lowercased, for the Save sheet's unit default
    var presetManager: PresetManagerWindow?

    var cardPath: String { defaults.string(forKey: "lastCard") ?? "" }
    var nandPath: String { defaults.string(forKey: "lastNand") ?? "" }

    func applicationDidFinishLaunching(_ note: Notification) {
        NSApp.setActivationPolicy(.regular)
        try? FileManager.default.createDirectory(atPath: logDir, withIntermediateDirectories: true)
        ensureUnitsFile(presetsPath)
        presets = loadPresets()
        buildWindow()
        cardField.stringValue = cardPath
        nandField.stringValue = nandPath
        refreshUnitLine()
        updateRunEnabled()
        window.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    // MARK: UI

    func label(_ s: String) -> NSTextField {
        let f = NSTextField(labelWithString: s)
        f.font = NSFont.systemFont(ofSize: 12)
        return f
    }

    func pathRow(initial: String, chooseExt: String, title: String) -> (NSView, NSTextField) {
        let row = NSStackView()
        row.orientation = .horizontal
        row.spacing = 8
        let l = label(title)
        l.widthAnchor.constraint(equalToConstant: 90).isActive = true
        let f = NSTextField(string: initial)
        f.isEditable = false
        f.isBordered = true
        f.widthAnchor.constraint(equalToConstant: 300).isActive = true
        f.lineBreakMode = .byTruncatingHead
        let b = NSButton(title: "Choose...", target: self, action: nil)
        row.addArrangedSubview(l)
        row.addArrangedSubview(f)
        row.addArrangedSubview(b)
        return (row, f)
    }

    func buildWindow() {
        let w = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 480, height: 430),
                          styleMask: [.titled, .closable, .miniaturizable, .resizable],
                          backing: .buffered, defer: false)
        w.title = "QY8 Emulator"
        w.delegate = self
        w.minSize = NSSize(width: 460, height: 360)
        w.center()
        window = w

        let root = NSStackView()
        root.orientation = .vertical
        root.alignment = .leading
        root.spacing = 10
        root.edgeInsets = NSEdgeInsets(top: 16, left: 16, bottom: 16, right: 16)
        root.translatesAutoresizingMaskIntoConstraints = false

        let prow = NSStackView()
        prow.spacing = 8
        prow.addArrangedSubview(label("Preset:"))
        presetPopup = NSPopUpButton()
        presetPopup.target = self
        presetPopup.action = #selector(presetChosen)
        rebuildPresetPopup()
        prow.addArrangedSubview(presetPopup)
        let saveButton = NSButton(title: "Save\u{2026}", target: self, action: #selector(saveTapped))
        let editButton = NSButton(title: "Edit\u{2026}", target: self, action: #selector(editTapped))
        prow.addArrangedSubview(saveButton)
        prow.addArrangedSubview(editButton)
        root.addArrangedSubview(prow)

        let (cardRow, cardF) = pathRow(initial: cardPath, chooseExt: "img", title: "Card image:")
        cardField = cardF
        (cardRow.subviews.last as? NSButton)?.target = self
        (cardRow.subviews.last as? NSButton)?.action = #selector(chooseCard)
        root.addArrangedSubview(cardRow)

        let (nandRow, nandF) = pathRow(initial: nandPath, chooseExt: "bin", title: "NAND dump:")
        nandField = nandF
        (nandRow.subviews.last as? NSButton)?.target = self
        (nandRow.subviews.last as? NSButton)?.action = #selector(chooseNand)
        root.addArrangedSubview(nandRow)

        unitLabel = label("Unit: pick a card and a NAND dump")
        unitLabel.textColor = .secondaryLabelColor
        root.addArrangedSubview(unitLabel)

        optionsToggle = NSButton(title: "\u{25B8} Options", target: self, action: #selector(toggleOptions))
        optionsToggle.isBordered = false
        optionsToggle.setButtonType(.momentaryChange)
        optionsToggle.alignment = .left
        root.addArrangedSubview(optionsToggle)

        optionsView = buildOptionsView()
        optionsView.isHidden = !defaults.bool(forKey: "optionsExpanded")
        optionsToggle.title = optionsView.isHidden ? "\u{25B8} Options" : "\u{25BE} Options"
        root.addArrangedSubview(optionsView)

        statusLabel = label("Status: idle")
        statusLabel.textColor = .secondaryLabelColor
        root.addArrangedSubview(statusLabel)

        let scroll = NSScrollView()
        scroll.hasVerticalScroller = true
        scroll.borderType = .bezelBorder
        scroll.translatesAutoresizingMaskIntoConstraints = false
        let tv = NSTextView()
        tv.isEditable = false
        tv.isSelectable = true
        tv.font = NSFont.monospacedSystemFont(ofSize: 11, weight: .regular)
        tv.textContainerInset = NSSize(width: 4, height: 4)
        tv.isVerticallyResizable = true
        tv.isHorizontallyResizable = false
        tv.textContainer?.widthTracksTextView = true
        scroll.documentView = tv
        logTextView = tv
        let scrollHeight = scroll.heightAnchor.constraint(equalToConstant: 180)
        scrollHeight.priority = .defaultHigh
        scrollHeight.isActive = true
        scroll.widthAnchor.constraint(greaterThanOrEqualToConstant: 440).isActive = true
        root.addArrangedSubview(scroll)

        let buttonRow = NSStackView()
        buttonRow.spacing = 10
        let helpButton = NSButton(title: "Help", target: self, action: #selector(showHelp))
        copyLogButton = NSButton(title: "Copy Log", target: self, action: #selector(copyLog))
        restartButton = NSButton(title: "Restart", target: self, action: #selector(restartTapped))
        restartButton.isEnabled = false
        runButton = NSButton(title: "Run", target: self, action: #selector(runTapped))
        runButton.keyEquivalent = "\r"
        buttonRow.addArrangedSubview(helpButton)
        buttonRow.addArrangedSubview(copyLogButton)
        buttonRow.addArrangedSubview(NSView())
        buttonRow.addArrangedSubview(restartButton)
        buttonRow.addArrangedSubview(runButton)
        root.addArrangedSubview(buttonRow)
        buttonRow.widthAnchor.constraint(equalTo: root.widthAnchor).isActive = true

        w.contentView = NSView()
        w.contentView!.addSubview(root)
        NSLayoutConstraint.activate([
            root.topAnchor.constraint(equalTo: w.contentView!.topAnchor),
            root.leadingAnchor.constraint(equalTo: w.contentView!.leadingAnchor),
            root.trailingAnchor.constraint(equalTo: w.contentView!.trailingAnchor),
            root.bottomAnchor.constraint(equalTo: w.contentView!.bottomAnchor),
        ])
    }

    func buildOptionsView() -> NSView {
        let v = NSStackView()
        v.orientation = .vertical
        v.alignment = .leading
        v.spacing = 6

        keepBox = NSButton(checkboxWithTitle: "Keep card changes", target: nil, action: nil)
        keepBox.state = defaults.bool(forKey: "optKeep") ? .on : .off
        verboseBox = NSButton(checkboxWithTitle: "Verbose log", target: nil, action: nil)
        verboseBox.state = defaults.bool(forKey: "optVerbose") ? .on : .off
        v.addArrangedSubview(keepBox)
        v.addArrangedSubview(verboseBox)

        let boardRow = NSStackView(); boardRow.spacing = 8
        boardRow.addArrangedSubview(label("Board:"))
        boardPopup = NSPopUpButton()
        boardPopup.addItems(withTitles: ["Auto", "qy8202na", "qy8202pb", "qy8602nb", "qy8652nb", "qy8600ne"])
        boardPopup.selectItem(withTitle: defaults.string(forKey: "optBoard") ?? "Auto")
        boardRow.addArrangedSubview(boardPopup)
        v.addArrangedSubview(boardRow)

        let modelRow = NSStackView(); modelRow.spacing = 8
        modelRow.addArrangedSubview(label("Model fallback:"))
        modelPopup = NSPopUpButton()
        modelPopup.addItems(withTitles: ["Auto", "ze0", "me0", "ze1"])
        modelPopup.selectItem(withTitle: defaults.string(forKey: "optModel") ?? "Auto")
        modelRow.addArrangedSubview(modelPopup)
        v.addArrangedSubview(modelRow)

        let extraRow = NSStackView(); extraRow.spacing = 8
        extraRow.addArrangedSubview(label("Extra flags:"))
        extraField = NSTextField(string: defaults.string(forKey: "optExtra") ?? "")
        extraField.widthAnchor.constraint(equalToConstant: 260).isActive = true
        extraRow.addArrangedSubview(extraField)
        v.addArrangedSubview(extraRow)

        return v
    }

    @objc func toggleOptions() {
        optionsView.isHidden.toggle()
        optionsToggle.title = optionsView.isHidden ? "\u{25B8} Options" : "\u{25BE} Options"
        defaults.set(!optionsView.isHidden, forKey: "optionsExpanded")
        window.layoutIfNeeded()
    }

    func rebuildPresetPopup() {
        presetPopup.removeAllItems()
        presetPopup.addItem(withTitle: "(choose files manually)")
        for p in presets { presetPopup.addItem(withTitle: p.label) }
    }

    @objc func presetChosen() {
        let idx = presetPopup.indexOfSelectedItem
        guard idx > 0, idx - 1 < presets.count else { return }
        let p = presets[idx - 1]
        cardField.stringValue = p.card
        nandField.stringValue = p.nand
        let (model, board, rest) = splitPresetExtra(p.extra)
        modelPopup.selectItem(withTitle: model ?? "Auto")
        boardPopup.selectItem(withTitle: board ?? "Auto")
        extraField.stringValue = rest
        defaults.set(p.card, forKey: "lastCard")
        defaults.set(p.nand, forKey: "lastNand")
        refreshUnitLine()
        updateRunEnabled()
    }

    @objc func chooseCard() {
        let panel = NSOpenPanel()
        panel.title = "Choose your card image"
        panel.allowedContentTypes = [UTType(filenameExtension: "img") ?? .data]
        panel.allowsOtherFileTypes = true
        if !cardField.stringValue.isEmpty {
            panel.directoryURL = URL(fileURLWithPath: cardField.stringValue).deletingLastPathComponent()
        }
        if panel.runModal() == .OK, let url = panel.url {
            cardField.stringValue = url.path
            defaults.set(url.path, forKey: "lastCard")
            refreshUnitLine()
            updateRunEnabled()
        }
    }

    @objc func chooseNand() {
        let panel = NSOpenPanel()
        panel.title = "Choose your NAND dump"
        panel.allowedContentTypes = [UTType(filenameExtension: "bin") ?? .data]
        panel.allowsOtherFileTypes = true
        if !nandField.stringValue.isEmpty {
            panel.directoryURL = URL(fileURLWithPath: nandField.stringValue).deletingLastPathComponent()
        }
        if panel.runModal() == .OK, let url = panel.url {
            nandField.stringValue = url.path
            defaults.set(url.path, forKey: "lastNand")
            refreshUnitLine()
            updateRunEnabled()
        }
    }

    func showError(_ msg: String) {
        let a = NSAlert()
        a.messageText = "Error"
        a.informativeText = msg
        a.alertStyle = .warning
        a.addButton(withTitle: "OK")
        a.runModal()
    }

    @objc func saveTapped() {
        guard !cardField.stringValue.isEmpty, !nandField.stringValue.isEmpty else { return }
        if pathHasSpace(cardField.stringValue) || pathHasSpace(nandField.stringValue) {
            showError("The card or NAND path has a space in it, and the presets file "
                + "can't quote paths yet. Move the file somewhere without one and try again.")
            return
        }

        let alert = NSAlert()
        alert.messageText = "Save preset"
        alert.informativeText = "Unit and variant name, as used with qy8boot UNIT VARIANT."
        alert.addButton(withTitle: "Save")
        alert.addButton(withTitle: "Cancel")

        let unitField = NSTextField(string: (detectedUnitCode ?? "").lowercased())
        unitField.placeholderString = "unit, e.g. qy8602nb"
        unitField.widthAnchor.constraint(equalToConstant: 200).isActive = true
        let variantField = NSTextField(string: "")
        variantField.placeholderString = "variant, e.g. iceland"
        variantField.widthAnchor.constraint(equalToConstant: 200).isActive = true

        let unitRow = NSStackView(views: [label("Unit:"), unitField]); unitRow.spacing = 8
        let variantRow = NSStackView(views: [label("Variant:"), variantField]); variantRow.spacing = 8
        let box = NSStackView(views: [unitRow, variantRow])
        box.orientation = .vertical
        box.spacing = 8
        alert.accessoryView = box
        alert.window.initialFirstResponder = unitField

        guard alert.runModal() == .alertFirstButtonReturn else { return }
        let unit = unitField.stringValue.trimmingCharacters(in: .whitespaces)
        let variant = variantField.stringValue.trimmingCharacters(in: .whitespaces)
        guard !unit.isEmpty, !variant.isEmpty else { return }
        guard !unit.contains(" "), !variant.contains(" ") else {
            showError("Unit and variant names can't contain spaces.")
            return
        }

        ensureUnitsFile(presetsPath)
        var lines = parseUnitsFile(presetsPath)
        let extra = joinPresetExtra(model: modelPopup.titleOfSelectedItem ?? "Auto",
                                     board: boardPopup.titleOfSelectedItem ?? "Auto",
                                     rest: extraField.stringValue)
        let newLine = PresetLine(raw: "", isPreset: true, unit: unit, variant: variant,
                                  card: cardField.stringValue, nand: nandField.stringValue, extra: extra)

        if let idx = lines.firstIndex(where: { $0.isPreset && $0.unit == unit && $0.variant == variant }) {
            let confirm = NSAlert()
            confirm.messageText = "Overwrite existing preset?"
            confirm.informativeText = "\(unit) \(variant) already exists."
            confirm.alertStyle = .warning
            confirm.addButton(withTitle: "Overwrite")
            confirm.addButton(withTitle: "Cancel")
            guard confirm.runModal() == .alertFirstButtonReturn else { return }
            lines[idx] = newLine
        } else {
            insertGrouped(&lines, newLine)
        }

        do {
            try writeUnitsFile(lines, to: presetsPath)
        } catch {
            showError("Couldn't save the presets file: \(error.localizedDescription)")
            return
        }
        presets = loadPresets()
        rebuildPresetPopup()
    }

    @objc func editTapped() {
        ensureUnitsFile(presetsPath)
        let mgr = PresetManagerWindow(owner: self)
        presetManager = mgr
        mgr.window.makeKeyAndOrderFront(nil)
    }

    func refreshUnitLine() {
        let card = cardField.stringValue, nand = nandField.stringValue
        guard !card.isEmpty, !nand.isEmpty else {
            unitLabel.stringValue = "Unit: pick a card and a NAND dump"
            return
        }
        unitLabel.stringValue = "Unit: checking..."
        DispatchQueue.global().async {
            let out = runCapture("/bin/bash", [bootSh, "--identify", card, nand])
            var unit = "Unit: unknown"
            var warn = ""
            var code: String?
            for line in out.split(separator: "\n") {
                if line.hasPrefix("UNIT: ") {
                    let body = line.dropFirst(6)
                    unit = "Unit: " + body
                    if let dot = body.range(of: " .") { code = String(body[body.startIndex..<dot.lowerBound]) }
                }
                if line.hasPrefix("WARN: ") { warn = "  " + line.dropFirst(6) }
            }
            DispatchQueue.main.async {
                self.unitLabel.stringValue = unit + warn
                self.detectedUnitCode = code
            }
        }
    }

    func updateRunEnabled() {
        runButton.isEnabled = !cardField.stringValue.isEmpty && !nandField.stringValue.isEmpty
    }

    @objc func showHelp() {
        let p = Process()
        p.executableURL = URL(fileURLWithPath: "/usr/bin/open")
        p.arguments = ["-e", quickstart]
        try? p.run()
    }

    func setStatus(_ s: String) { statusLabel.stringValue = "Status: " + s }

    func appendLog(_ s: String) {
        guard let storage = logTextView.textStorage else { return }
        storage.append(NSAttributedString(string: s + "\n",
            attributes: [.font: NSFont.monospacedSystemFont(ofSize: 11, weight: .regular)]))
        if storage.length > 300_000 { storage.deleteCharacters(in: NSRange(location: 0, length: storage.length / 2)) }
        logTextView.scrollToEndOfDocument(nil)
    }

    @objc func runTapped() {
        if isRunning { stopTapped(); return }

        defaults.set(keepBox.state == .on, forKey: "optKeep")
        defaults.set(verboseBox.state == .on, forKey: "optVerbose")
        defaults.set(boardPopup.titleOfSelectedItem, forKey: "optBoard")
        defaults.set(modelPopup.titleOfSelectedItem, forKey: "optModel")
        defaults.set(extraField.stringValue, forKey: "optExtra")

        var args = [cardField.stringValue, nandField.stringValue]
        if keepBox.state == .on { args.append("--keep") }
        if verboseBox.state == .on { args.append("--verbose") }
        if let b = boardPopup.titleOfSelectedItem, b != "Auto" { args += ["--board", b] }
        if let m = modelPopup.titleOfSelectedItem, m != "Auto" { args += ["--model", m] }
        if !extraField.stringValue.isEmpty { args += ["--extra", extraField.stringValue] }
        args += ["--workdir", logDir + "/work"]

        logTextView.textStorage?.setAttributedString(NSAttributedString(string: ""))
        setStatus("booting...")
        runButton.isEnabled = false
        runButton.title = "Run"
        restartButton.isEnabled = false
        qemuPid = nil

        let p = Process()
        p.executableURL = URL(fileURLWithPath: "/bin/bash")
        p.arguments = [bootSh] + args
        let pipe = Pipe()
        p.standardOutput = pipe
        p.standardError = pipe

        var logTail: [String] = []

        // boot.sh's stdout is now only ever its own protocol lines (UNIT/BOARD/
        // WORK/PID/READY/EXITED/FAIL, plus a static LOG: tail on failure) - the
        // live log pane reads console.log/qemu.log directly (see pump/startTailing)
        // so two writers never race on one fd and garble a line mid-word.
        pipe.fileHandleForReading.readabilityHandler = { [weak self] handle in
            let data = handle.availableData
            guard !data.isEmpty, let text = String(data: data, encoding: .utf8) else { return }
            guard let self = self else { return }
            for line in text.split(separator: "\n", omittingEmptySubsequences: false) where !line.isEmpty {
                if line.hasPrefix("LOG: ") {
                    logTail.append(String(line.dropFirst(5)))
                } else if line.hasPrefix("WORK: ") {
                    let dir = String(line.dropFirst(6))
                    DispatchQueue.main.async { self.startTailing(dir) }
                } else if line.hasPrefix("PID: ") {
                    self.qemuPid = Int32(line.dropFirst(5))
                } else if line == "READY" {
                    DispatchQueue.main.async {
                        self.isRunning = true
                        self.setStatus("running")
                        self.runButton.title = "Stop"
                        self.runButton.isEnabled = true
                        self.restartButton.isEnabled = true
                    }
                } else if line.hasPrefix("EXITED: ") {
                    let code = String(line.dropFirst(8))
                    DispatchQueue.main.async {
                        self.isRunning = false
                        self.stopTailing()
                        self.setStatus("exited (code \(code))")
                        self.runButton.title = "Run"
                        self.runButton.isEnabled = true
                        self.restartButton.isEnabled = true
                    }
                } else if line.hasPrefix("FAIL: ") {
                    let reason = String(line.dropFirst(6))
                    DispatchQueue.main.async { self.stopTailing(); self.showFailure(reason, logTail) }
                }
            }
        }

        p.terminationHandler = { [weak self] _ in
            pipe.fileHandleForReading.readabilityHandler = nil
            DispatchQueue.main.async {
                guard let self = self else { return }
                self.isRunning = false
                self.runButton.isEnabled = true
                self.runButton.title = "Run"
            }
        }
        runProcess = p
        try? p.run()
    }

    func startTailing(_ dir: String) {
        consoleOffset = 0
        qemuLogOffset = 0
        tailTimer?.invalidate()
        let timer = Timer(timeInterval: 0.4, repeats: true) { [weak self] _ in
            self?.pump(dir + "/console.log", isConsole: true)
            self?.pump(dir + "/qemu.log", isConsole: false)
        }
        RunLoop.main.add(timer, forMode: .common)
        tailTimer = timer
    }

    func stopTailing() {
        tailTimer?.invalidate()
        tailTimer = nil
    }

    func pump(_ path: String, isConsole: Bool) {
        guard let fh = FileHandle(forReadingAtPath: path) else { return }
        defer { try? fh.close() }
        fh.seek(toFileOffset: isConsole ? consoleOffset : qemuLogOffset)
        let data = fh.readDataToEndOfFile()
        guard !data.isEmpty else { return }
        if isConsole { consoleOffset += UInt64(data.count) } else { qemuLogOffset += UInt64(data.count) }
        guard let text = String(data: data, encoding: .utf8) else { return }
        appendLog(text.hasSuffix("\n") ? String(text.dropLast()) : text)
    }

    @objc func stopTapped() {
        if let pid = qemuPid { kill(pid, SIGTERM) }
        runButton.isEnabled = false
        setStatus("stopping...")
    }

    @objc func restartTapped() {
        if isRunning {
            if let pid = qemuPid { kill(pid, SIGTERM) }
            // the termination handler's EXITED/EXITED line will have landed before
            // the user can click again in practice; a short delay keeps it simple
            DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) { self.runTapped() }
        } else {
            runTapped()
        }
    }

    @objc func copyLog() {
        let pasteboard = NSPasteboard.general
        pasteboard.clearContents()
        pasteboard.setString(logTextView.string, forType: .string)
    }

    func showFailure(_ reason: String, _ log: [String]) {
        isRunning = false
        runButton.isEnabled = true
        runButton.title = "Run"
        restartButton.isEnabled = true
        setStatus("failed")
        window.makeKeyAndOrderFront(nil)
        let alert = NSAlert()
        alert.messageText = "The emulator did not start"
        var info = reason
        if !log.isEmpty { info += "\n\n" + log.suffix(12).joined(separator: "\n") }
        alert.informativeText = info
        alert.alertStyle = .warning
        alert.addButton(withTitle: "OK")
        alert.runModal()
    }

    func windowShouldClose(_ sender: NSWindow) -> Bool {
        guard isRunning else { return true }
        let alert = NSAlert()
        alert.messageText = "Stop the emulator?"
        alert.informativeText = "QY8 Emulator is still running."
        alert.addButton(withTitle: "Stop and Close")
        alert.addButton(withTitle: "Cancel")
        if alert.runModal() == .alertFirstButtonReturn {
            if let pid = qemuPid { kill(pid, SIGTERM) }
            return true
        }
        return false
    }
}

func runConfigApp() {
    let delegate = ConfigWindow()
    let app = NSApplication.shared
    app.delegate = delegate
    app.run()
}

// Preset manager: a table over the units file plus Delete, Rename, Make
// default (reorders within the unit's own block) and Show in Finder.
final class PresetManagerWindow: NSObject, NSWindowDelegate, NSTableViewDataSource, NSTableViewDelegate {
    weak var owner: ConfigWindow?
    var window: NSWindow!
    var table: NSTableView!
    var lines: [PresetLine] = []
    var presetIndexes: [Int] = []   // indexes into `lines` that are presets, in file order

    init(owner: ConfigWindow) {
        self.owner = owner
        super.init()
        reload()
        buildWindow()
    }

    func reload() {
        lines = parseUnitsFile(presetsPath)
        presetIndexes = lines.indices.filter { lines[$0].isPreset }
    }

    func buildWindow() {
        let w = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 620, height: 340),
                          styleMask: [.titled, .closable, .miniaturizable, .resizable],
                          backing: .buffered, defer: false)
        w.title = "Presets"
        w.delegate = self
        w.center()
        window = w

        let root = NSStackView()
        root.orientation = .vertical
        root.edgeInsets = NSEdgeInsets(top: 12, left: 12, bottom: 12, right: 12)
        root.spacing = 8
        root.translatesAutoresizingMaskIntoConstraints = false

        let scroll = NSScrollView()
        scroll.hasVerticalScroller = true
        scroll.borderType = .bezelBorder
        let t = NSTableView()
        t.usesAlternatingRowBackgroundColors = true
        for (title, width) in [("Unit", 90.0), ("Variant", 90.0), ("Card", 140.0), ("NAND", 140.0), ("Flags", 100.0)] {
            let col = NSTableColumn(identifier: NSUserInterfaceItemIdentifier(title))
            col.title = title
            col.width = width
            t.addTableColumn(col)
        }
        t.dataSource = self
        t.delegate = self
        scroll.documentView = t
        table = t
        scroll.heightAnchor.constraint(greaterThanOrEqualToConstant: 220).isActive = true
        root.addArrangedSubview(scroll)
        scroll.widthAnchor.constraint(equalTo: root.widthAnchor).isActive = true

        let buttons = NSStackView()
        buttons.spacing = 8
        buttons.addArrangedSubview(NSButton(title: "Delete", target: self, action: #selector(deleteTapped)))
        buttons.addArrangedSubview(NSButton(title: "Rename", target: self, action: #selector(renameTapped)))
        buttons.addArrangedSubview(NSButton(title: "Make default", target: self, action: #selector(makeDefaultTapped)))
        buttons.addArrangedSubview(NSButton(title: "Show file in Finder", target: self, action: #selector(showInFinder)))
        root.addArrangedSubview(buttons)

        w.contentView = NSView()
        w.contentView!.addSubview(root)
        NSLayoutConstraint.activate([
            root.topAnchor.constraint(equalTo: w.contentView!.topAnchor),
            root.leadingAnchor.constraint(equalTo: w.contentView!.leadingAnchor),
            root.trailingAnchor.constraint(equalTo: w.contentView!.trailingAnchor),
            root.bottomAnchor.constraint(equalTo: w.contentView!.bottomAnchor),
        ])
    }

    func numberOfRows(in tableView: NSTableView) -> Int { presetIndexes.count }

    func tableView(_ tableView: NSTableView, viewFor column: NSTableColumn?, row: Int) -> NSView? {
        let p = lines[presetIndexes[row]]
        let text: String
        switch column?.identifier.rawValue {
        case "Unit": text = p.unit
        case "Variant": text = p.variant
        case "Card": text = (p.card as NSString).lastPathComponent
        case "NAND": text = (p.nand as NSString).lastPathComponent
        case "Flags": text = p.extra.isEmpty ? "-" : p.extra
        default: text = ""
        }
        let field = NSTextField(labelWithString: text)
        field.lineBreakMode = .byTruncatingMiddle
        return field
    }

    func showError(_ msg: String) {
        let a = NSAlert(); a.messageText = "Error"; a.informativeText = msg
        a.alertStyle = .warning; a.addButton(withTitle: "OK"); a.runModal()
    }

    // Writes the file, then tells both this window and the main one to
    // reload, so the preset popup reflects whatever just changed.
    func persist() {
        do {
            try writeUnitsFile(lines, to: presetsPath)
        } catch {
            showError("Couldn't save: \(error.localizedDescription)")
        }
        reload()
        table.reloadData()
        owner?.presets = loadPresets()
        owner?.rebuildPresetPopup()
    }

    @objc func deleteTapped() {
        let row = table.selectedRow
        guard row >= 0, row < presetIndexes.count else { return }
        let idx = presetIndexes[row]
        let p = lines[idx]
        let confirm = NSAlert()
        confirm.messageText = "Delete preset \(p.unit) \(p.variant)?"
        confirm.alertStyle = .warning
        confirm.addButton(withTitle: "Delete")
        confirm.addButton(withTitle: "Cancel")
        guard confirm.runModal() == .alertFirstButtonReturn else { return }
        lines.remove(at: idx)
        persist()
    }

    @objc func renameTapped() {
        let row = table.selectedRow
        guard row >= 0, row < presetIndexes.count else { return }
        let idx = presetIndexes[row]

        let alert = NSAlert()
        alert.messageText = "Rename variant"
        alert.informativeText = "New variant name for \(lines[idx].unit) \(lines[idx].variant)."
        alert.addButton(withTitle: "Rename")
        alert.addButton(withTitle: "Cancel")
        let field = NSTextField(string: lines[idx].variant)
        field.frame = NSRect(x: 0, y: 0, width: 220, height: 24)
        alert.accessoryView = field
        alert.window.initialFirstResponder = field
        guard alert.runModal() == .alertFirstButtonReturn else { return }

        let newName = field.stringValue.trimmingCharacters(in: .whitespaces)
        guard !newName.isEmpty, !newName.contains(" ") else {
            showError("Variant names can't be empty or contain spaces.")
            return
        }
        let unit = lines[idx].unit
        let dup = presetIndexes.contains { $0 != idx && lines[$0].unit == unit && lines[$0].variant == newName }
        guard !dup else {
            showError("\(unit) \(newName) already exists.")
            return
        }
        lines[idx].variant = newName
        persist()
    }

    @objc func makeDefaultTapped() {
        let row = table.selectedRow
        guard row >= 0, row < presetIndexes.count else { return }
        let idx = presetIndexes[row]
        let unit = lines[idx].unit
        let groupIdxs = presetIndexes.filter { lines[$0].unit == unit }
        guard let firstIdx = groupIdxs.first, firstIdx != idx else { return }
        let moved = lines.remove(at: idx)
        lines.insert(moved, at: firstIdx)
        persist()
    }

    @objc func showInFinder() {
        NSWorkspace.shared.activateFileViewerSelecting([URL(fileURLWithPath: presetsPath)])
    }

    func windowShouldClose(_ sender: NSWindow) -> Bool { true }
}
