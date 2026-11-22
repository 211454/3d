import AVFoundation
import CoreMotion
import Foundation
import SwiftUI
import Speech

enum MeasurementMode: String, CaseIterable, Identifiable {
    case camber, toe
    var id: String { rawValue }
    func label(_ german: Bool) -> String { self == .camber ? (german ? "Sturz" : "Развал") : (german ? "Spur (Test)" : "Схождение (тест)") }
}

enum Wheel: String, CaseIterable, Identifiable {
    case frontLeft, frontRight, rearLeft, rearRight

    var id: String { rawValue }
    func label(german: Bool) -> String {
        switch self {
        case .frontLeft: german ? "Vorne links" : "Переднее левое"
        case .frontRight: german ? "Vorne rechts" : "Переднее правое"
        case .rearLeft: german ? "Hinten links" : "Заднее левое"
        case .rearRight: german ? "Hinten rechts" : "Заднее правое"
        }
    }
}

@MainActor
final class AlignmentModel: ObservableObject {
    @Published private(set) var angle: Double?
    @Published private(set) var liveAngle: Double?
    @Published private(set) var running = false
    @Published private(set) var unavailable = false
    @Published private(set) var stable = false
    @Published private(set) var calibrated = false
    @Published private(set) var correctPosition = false
    @Published private(set) var movementDetected = false
    @Published private(set) var saved: [Wheel: Double] = [:]
    @Published private(set) var toeSaved: [Wheel: Double] = [:]
    @Published private(set) var mode: MeasurementMode = .camber
    @Published private(set) var stableSeconds = 0.0
    private var stableSince: TimeInterval?
    private var toeReference: Double?
    private var toeZero = 0.0
    var frontDifference: Double? { guard let l = saved[.frontLeft], let r = saved[.frontRight] else { return nil }; return l - r }
    var rearDifference: Double? { guard let l = saved[.rearLeft], let r = saved[.rearRight] else { return nil }; return l - r }

    private let motion = CMMotionManager()
    private var timer: Timer?
    private var samples: [Double] = []
    private var lastTimestamp: TimeInterval?
    private var zero = 0.0
    private var camberCalibrated = false
    private let storageKey = "autoklinik3d.measurements.v1"

    init() { restoreMeasurements() }

    private struct StoredMeasurements: Codable {
        let camber: [String: Double]
        let toe: [String: Double]
    }

    private func persistMeasurements() {
        let record = StoredMeasurements(camber: Dictionary(uniqueKeysWithValues: saved.map { ($0.key.rawValue, $0.value) }), toe: Dictionary(uniqueKeysWithValues: toeSaved.map { ($0.key.rawValue, $0.value) }))
        if let data = try? JSONEncoder().encode(record) { UserDefaults.standard.set(data, forKey: storageKey) }
    }

    private func restoreMeasurements() {
        guard let data = UserDefaults.standard.data(forKey: storageKey), let record = try? JSONDecoder().decode(StoredMeasurements.self, from: data) else { return }
        saved = Dictionary(uniqueKeysWithValues: record.camber.compactMap { key, value in Wheel(rawValue: key).map { ($0, value) } })
        toeSaved = Dictionary(uniqueKeysWithValues: record.toe.compactMap { key, value in Wheel(rawValue: key).map { ($0, value) } })
    }

    func setMode(_ newMode: MeasurementMode) {
        mode = newMode
        calibrated = newMode == .camber ? camberCalibrated : toeReference != nil
        resetSampling()
    }

    func start() {
        guard !running else { return }
        guard motion.isDeviceMotionAvailable else {
            unavailable = true
            return
        }
        unavailable = false
        resetSampling()
        lastTimestamp = nil
        motion.deviceMotionUpdateInterval = 0.05
        motion.startDeviceMotionUpdates()
        running = true
        timer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.readMotion() }
        }
    }

    func stop() {
        timer?.invalidate()
        timer = nil
        motion.stopDeviceMotionUpdates()
        running = false
        liveAngle = nil
        correctPosition = false
        resetSampling()
    }

    func resetSampling() {
        samples.removeAll(keepingCapacity: true)
        stable = false
        stableSeconds = 0
        stableSince = nil
        angle = nil
    }

    func calibrate() {
        guard stable, let angle else { return }
        if mode == .camber {
            zero += angle
            camberCalibrated = true
            saved.removeAll()
            persistMeasurements()
        } else {
            guard let heading = motion.deviceMotion?.attitude.yaw else { return }
            toeReference = heading
            toeZero = heading
            toeSaved.removeAll()
            persistMeasurements()
        }
        calibrated = true
        resetSampling()
    }

    func calibrateKeepingMeasurements() {
        guard stable, let angle else { return }
        if mode == .camber { zero += angle; camberCalibrated = true }
        else if let heading = motion.deviceMotion?.attitude.yaw { toeReference = heading; toeZero = heading }
        calibrated = true
        resetSampling()
    }

    func unlock(_ wheel: Wheel) {
        if mode == .camber { saved.removeValue(forKey: wheel) }
        else { toeSaved.removeValue(forKey: wheel) }
        persistMeasurements()
        resetSampling()
    }

    func save(_ wheel: Wheel, guidedCapture: Bool = false) {
        guard (mode == .camber ? saved[wheel] : toeSaved[wheel]) == nil else { return }

        guard running, (calibrated || guidedCapture), correctPosition, stable, let angle, angle.isFinite else { return }
        if mode == .camber { saved[wheel] = angle } else { toeSaved[wheel] = angle }
        persistMeasurements()
        resetSampling()
    }

    private func readMotion() {
        guard running else { return }
        guard let sample = motion.deviceMotion,
              sample.timestamp > 0 else {
            correctPosition = false
            resetSampling()
            return
        }
        guard lastTimestamp != sample.timestamp else { return }
        lastTimestamp = sample.timestamp

        let gravity = sample.gravity
        let raw = mode == .camber
            ? atan2(gravity.x, -gravity.y) * 180 / .pi
            : atan2(sin(sample.attitude.yaw - toeZero), cos(sample.attitude.yaw - toeZero)) * 180 / .pi
        guard raw.isFinite else { return }
        let offset = mode == .camber ? zero : 0
        let measured = mode == .camber ? raw - offset : raw
        liveAngle = measured
        // Portrait, top edge up, screen facing away from the vehicle.
        correctPosition = hypot(gravity.x, gravity.y) > 0.75
        let rotation = sample.rotationRate
        let acceleration = sample.userAcceleration
        movementDetected = hypot(hypot(rotation.x, rotation.y), rotation.z) >= 0.15 || hypot(hypot(acceleration.x, acceleration.y), acceleration.z) >= 0.12
        guard correctPosition,
              hypot(hypot(rotation.x, rotation.y), rotation.z) < 0.15,
              hypot(hypot(acceleration.x, acceleration.y), acceleration.z) < 0.12 else {
            resetSampling()
            return
        }

        samples.append(measured)
        if samples.count > 20 { samples.removeFirst() }
        let minimum = samples.min() ?? raw
        let maximum = samples.max() ?? raw
        angle = samples.reduce(0, +) / Double(samples.count)
        stable = samples.count == 20 && maximum - minimum <= 0.30
        if stable {
            if stableSince == nil { stableSince = sample.timestamp }
            stableSeconds = max(0, sample.timestamp - (stableSince ?? sample.timestamp))
        } else {
            stableSince = nil
            stableSeconds = 0
        }
    }
}


/// Experimental rolling capture. Measures repeatable phone tilt changes, NOT validated wheel alignment.
@MainActor final class RollingCapture: ObservableObject {
    @Published private(set) var first: Double?
    @Published private(set) var second: Double?
    @Published private(set) var phase = 0
    @Published private(set) var stable = false
    private let motion = CMMotionManager()
    private var timer: Timer?
    private var values: [Double] = []
    private var movementObserved = false

    var difference: Double? {
        guard let first, let second else { return nil }
        return second - first
    }

    func start() {
        stop()
        first = nil
        second = nil
        phase = 1
        movementObserved = false
        values = []
        guard motion.isDeviceMotionAvailable else { phase = 0; return }
        motion.deviceMotionUpdateInterval = 0.05
        motion.startDeviceMotionUpdates()
        timer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.sample() }
        }
    }

    func stop() {
        timer?.invalidate()
        timer = nil
        motion.stopDeviceMotionUpdates()
    }

    private func sample() {
        guard let data = motion.deviceMotion, phase == 1 || phase == 2 else { return }
        let rotation = data.rotationRate
        let acceleration = data.userAcceleration
        let moving = sqrt(rotation.x*rotation.x + rotation.y*rotation.y + rotation.z*rotation.z) > 0.15 ||
            sqrt(acceleration.x*acceleration.x + acceleration.y*acceleration.y + acceleration.z*acceleration.z) > 0.12
        if moving {
            if phase == 2 { movementObserved = true }
            values.removeAll()
            stable = false
            return
        }
        if phase == 2 && !movementObserved { return }
        let gravity = data.gravity
        let tilt = atan2(gravity.z, hypot(gravity.x, gravity.y)) * 180 / Double.pi
        guard tilt.isFinite else { return }
        values.append(tilt)
        if values.count > 40 { values.removeFirst() }
        guard values.count == 40 else { return }
        let spread = (values.max() ?? tilt) - (values.min() ?? tilt)
        stable = spread < 0.2
        guard stable else { return }
        let mean = values.reduce(0,+) / Double(values.count)
        if phase == 1 {
            first = mean
            phase = 2
            values.removeAll()
        } else {
            second = mean
            phase = 3
            stop()
        }
    }
}


enum SpokenCommand { case none, zero, start, next, begin, adjust }

@MainActor
final class VoiceCommandController: ObservableObject {
    @Published private(set) var listening = false
    @Published private(set) var lastHeard = ""
    @Published private(set) var lastCommand: SpokenCommand = .none
    @Published private(set) var commandSequence = 0
    private let engine = AVAudioEngine()
    private var request: SFSpeechAudioBufferRecognitionRequest?
    private var task: SFSpeechRecognitionTask?
    private var lastProcessed = ""
    private var tapInstalled = false

    func start(locale: String) {
        guard !listening else { return }
        SFSpeechRecognizer.requestAuthorization { [weak self] status in
            Task { @MainActor in
                guard status == .authorized else {
                    self?.lastHeard = "Нет разрешения на распознавание речи"
                    return
                }
                self?.begin(locale: locale)
            }
        }
    }

    private func begin(locale: String) {
        guard let recognizer = SFSpeechRecognizer(locale: Locale(identifier: locale)), recognizer.isAvailable else {
            lastHeard = "Распознавание речи недоступно"
            return
        }
        do {
            let session = AVAudioSession.sharedInstance()
            try session.setCategory(.playAndRecord, mode: .default, options: [.defaultToSpeaker, .duckOthers])
            try session.setActive(true)
            let input = engine.inputNode
            let format = input.outputFormat(forBus: 0)
            let req = SFSpeechAudioBufferRecognitionRequest()
            req.shouldReportPartialResults = true
            request = req
            input.installTap(onBus: 0, bufferSize: 1024, format: format) { buffer, _ in
                req.append(buffer)
            }
            tapInstalled = true
            engine.prepare()
            try engine.start()
            listening = true
            task = recognizer.recognitionTask(with: req) { [weak self] result, error in
                Task { @MainActor in
                    if let result {
                        let words = result.bestTranscription.formattedString.lowercased()
                        self?.lastHeard = words
                        self?.process(words)
                    }
                    if error != nil || result?.isFinal == true { self?.stop() }
                }
            }
        } catch {
            lastHeard = "Ошибка микрофона: \(error.localizedDescription)"
            stop()
        }
    }

    private func process(_ words: String) {
        guard words != lastProcessed else { return }
        let command: SpokenCommand
        if words.contains("установи ноль") || words.contains("установить ноль") || words.contains("nullpunkt") { command = .zero }
        else if words.contains("начать измерение") || words.contains("начни измерение") || words.contains("messung starten") { command = .start }
        else if words.contains("следующее колесо") || words.contains("nächstes rad") { command = .next }
        else if words.contains("начать проверку") || words.contains("prüfung starten") { command = .begin }
        else if words.contains("начать регулировку") || words.contains("einstellung starten") { command = .adjust }
        else { return }
        lastProcessed = words
        lastCommand = command
        commandSequence += 1
    }

    func stop() {
        if engine.isRunning { engine.stop() }
        if tapInstalled { engine.inputNode.removeTap(onBus: 0); tapInstalled = false }
        request?.endAudio()
        task?.cancel()
        task = nil
        request = nil
        listening = false
        try? AVAudioSession.sharedInstance().setActive(false)
    }
}

struct ContentView: View {
    @StateObject private var rolling = RollingCapture()
    @StateObject private var model = AlignmentModel()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("alignmentGerman") private var german = false
    @State private var wheel = Wheel.frontLeft
    @State private var mode = MeasurementMode.camber
    @State private var selectedVehicle = "VW Golf 7"
    @State private var activeWheel: Wheel?
    @State private var guided = false
    @State private var adjusting = false
    @State private var completed = false
    @State private var awaitingMovement = false
    @State private var speech = AVSpeechSynthesizer()
    @StateObject private var voiceControl = VoiceCommandController()
    @State private var zeroConfirmed = false
    @State private var measurementStarted = false
    private let wheelOrder: [Wheel] = [.frontLeft, .frontRight, .rearLeft, .rearRight]

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 12) {
                    HStack {
                        Text("AUTOKLINIK").font(.title2.bold()).foregroundStyle(.white)
                        Text("3D").font(.title.bold()).foregroundStyle(.cyan)
                        Spacer()
                        Picker("Язык", selection: $german) {
                            Text("RU").tag(false)
                            Text("DE").tag(true)
                        }.pickerStyle(.segmented).frame(width: 110)
                    }
                    Text(german ? "3D-ACHSVERMESSUNG · SENSORTEST" : "РАЗВАЛ · СХОЖДЕНИЕ · ТЕСТ ДАТЧИКОВ")
                        .font(.caption.bold()).foregroundStyle(.cyan)
                    Picker(german ? "Fahrzeug" : "Автомобиль", selection: $selectedVehicle) {
                        ForEach(AlignmentDemoCatalog.models, id: \.self) { Text($0).tag($0) }
                    }
                    Picker(german ? "Messung" : "Измерение", selection: $mode) {
                        ForEach(MeasurementMode.allCases) { item in Text(item.label(german)).tag(item) }
                    }.pickerStyle(.segmented)
                    HStack {
                        Image(systemName: "waveform").foregroundStyle(.cyan)
                        Text(voiceControl.listening ? (german ? "Sprachsteuerung aktiv" : "Голосовое управление включено") : (german ? "Sprachsteuerung aus" : "Голосовое управление выключено"))
                            .font(.caption)
                        Spacer()
                        Button(voiceControl.listening ? (german ? "Aus" : "Выключить") : (german ? "Mikrofon" : "Микрофон")) {
                            if voiceControl.listening { voiceControl.stop() }
                            else { voiceControl.start(locale: german ? "de-DE" : "ru-RU") }
                        }.buttonStyle(.bordered)
                    }
                    Text(voiceControl.lastHeard.isEmpty ? (german ? "Befehle: Nullpunkt, Messung starten, nächstes Rad" : "Команды: установить ноль, начать измерение, следующее колесо") : voiceControl.lastHeard)
                        .font(.caption2).foregroundStyle(.secondary)
                    Text(german ? "Nur Sensortest — Spurwerte nicht validiert" : "Тест датчиков: точность схождения не подтверждена")
                        .font(.caption).foregroundStyle(.orange)
                    Text(guided ? (activeWheel?.label(german: german) ?? "") : completed ? (german ? "Messungen abgeschlossen" : "Проверка завершена") : adjusting ? (german ? "Einstellmodus" : "Режим регулировки") : (german ? "Bereit zur Prüfung" : "Готово к проверке"))
                        .font(.headline).foregroundStyle(.white)
                    Text(model.liveAngle.map(formatted) ?? "—")
                        .font(.system(size: 32, weight: .bold, design: .rounded))
                        .foregroundStyle(.cyan).monospacedDigit()
                    carDiagram
                    if guided {
                        Text(awaitingMovement ? (german ? "Telefon zum nächsten Rad bewegen" : "Перенесите телефон на следующее колесо") : !zeroConfirmed ? (german ? "Nullpunkt setzen" : "Установите ноль") : !measurementStarted ? (german ? "Messung starten" : "Начните измерение") : (german ? "Vier Sekunden stabil halten" : "Держите неподвижно четыре секунды"))
                            .font(.callout)
                        if measurementStarted { Text(String(format: "%.1f / 4.0 s", model.stableSeconds)).monospacedDigit() }
                    }
                    Text(status).font(.caption).foregroundStyle(.secondary)
                    Text(german ? "Türkis: gespeichert; keine Bewertung ohne geprüfte Sollwerte." : "Голубой: сохранено. Без заводских допусков оценка недоступна.")
                        .font(.caption2).foregroundStyle(.secondary)
                }.padding(14)
            }
            .background(
                LinearGradient(colors: [Color(red: 0.015, green: 0.055, blue: 0.11), .black, Color(red: 0.015, green: 0.09, blue: 0.17)], startPoint: .top, endPoint: .bottom)
                    .ignoresSafeArea()
            )
            .navigationBarHidden(true)
            .preferredColorScheme(.dark)
        }
        .onChange(of: wheel) { _ in model.resetSampling() }
        .onChange(of: model.stableSeconds) { _ in advanceGuidedIfReady() }
        .onChange(of: model.movementDetected) { moving in
            if guided && awaitingMovement && moving {
                awaitingMovement = false
                zeroConfirmed = false
                measurementStarted = false
                model.resetSampling()
                speak((german ? "Telefon befestigen und Nullpunkt setzen: " : "Закрепите телефон и установите ноль: ") + (activeWheel?.label(german: german) ?? ""))
            }
        }
        .onChange(of: voiceControl.commandSequence) { _ in
            switch voiceControl.lastCommand {
            case .zero: performZero()
            case .start: performStartMeasurement()
            case .next: if awaitingMovement { awaitingMovement = false; zeroConfirmed = false; measurementStarted = false; model.resetSampling() }
            case .begin: if !guided && !completed && !adjusting { beginGuided() }
            case .adjust: if completed { beginAdjustment() }
            case .none: break
            }
        }
        .onChange(of: mode) { value in activeWheel = nil; model.setMode(value) }
        .onChange(of: scenePhase) { phase in
            if phase != .active { model.stop(); rolling.stop(); voiceControl.stop() }
        }
    }

    private func performZero() {
        guard guided, !awaitingMovement, !zeroConfirmed, model.stable else { return }
        // Calibrate the mount once per session; never erase previously saved wheel readings.
        if !model.calibrated { model.calibrateKeepingMeasurements() }
        zeroConfirmed = true
        speak(german ? "Nullpunkt gesetzt. Messung starten." : "Ноль установлен. Начните измерение.")
    }

    private func performStartMeasurement() {
        guard guided, zeroConfirmed, !awaitingMovement else { return }
        measurementStarted = true
        model.resetSampling()
        speak(german ? "Messung läuft. Vier Sekunden ruhig halten." : "Измерение началось. Держите телефон неподвижно четыре секунды.")
    }

    private func beginAdjustment() {
        guard completed else { return }
        adjusting = true
        completed = false
        speak(german ? "Einstellmodus. Ausgangswerte gespeichert." : "Режим регулировки. Исходные измерения сохранены.")
    }

    private var centerAction: some View {
        Group {
            if completed && !adjusting {
                Button(german ? "Einstellung starten" : "Начать регулировку") { beginAdjustment() }
            } else if guided && !awaitingMovement && !zeroConfirmed {
                Button(german ? "Nullpunkt" : "Установить ноль") { performZero() }
                    .disabled(!model.stable)
            } else if guided && !awaitingMovement && !measurementStarted {
                Button(german ? "Messung starten" : "Начать измерение") { performStartMeasurement() }
            } else if !guided && !adjusting {
                Button(german ? "Prüfung starten" : "Начать проверку") { beginGuided() }
            } else {
                Text(adjusting ? (german ? "EINSTELLUNG" : "РЕГУЛИРОВКА") : (german ? "Messung läuft" : "Идёт измерение"))
                    .font(.caption.bold()).foregroundStyle(.cyan)
            }
        }
        .font(.system(size: 12, weight: .bold))
        .buttonStyle(.borderedProminent)
        .tint(.blue)
        .lineLimit(2)
        .minimumScaleFactor(0.75)
        .frame(maxWidth: 170)
    }

    private func speak(_ message: String) {
        speech.stopSpeaking(at: .immediate)
        let utterance = AVSpeechUtterance(string: message)
        utterance.voice = AVSpeechSynthesisVoice(language: german ? "de-DE" : "ru-RU")
        speech.speak(utterance)
    }

    private func beginGuided() {
        guided = true
        completed = false
        adjusting = false
        awaitingMovement = false
        zeroConfirmed = false
        measurementStarted = false
        mode = .camber
        model.setMode(.camber)
        activeWheel = wheelOrder.first(where: { model.saved[$0] == nil })
        if activeWheel == nil { guided = false; completed = true; return }
        wheel = activeWheel ?? .frontLeft
        model.resetSampling()
        if !model.running { model.start() }
        speak(german ? "Telefon am linken Vorderrad befestigen. Nullpunkt prüfen." : "Установите телефон на переднее левое колесо. Проверьте установку нуля.")
    }

    private func advanceGuidedIfReady() {
        guard guided, measurementStarted, zeroConfirmed, !awaitingMovement, model.stableSeconds >= 4,
              let current = activeWheel else { return }
        model.save(current, guidedCapture: true)
        guard model.saved[current] != nil else { return }
        let next = wheelOrder.first(where: { model.saved[$0] == nil })
        if let next {
            activeWheel = next
            wheel = next
            awaitingMovement = true
            measurementStarted = false
            zeroConfirmed = false
            speak((german ? "Gespeichert. Telefon abnehmen und am nächsten Rad befestigen: " : "Измерение сохранено. Снимите телефон и установите на другое колесо: ") + next.label(german: german))
        } else {
            guided = false
            completed = true
            activeWheel = nil
            speak(german ? "Vier Messungen gespeichert. Ergebnisse prüfen." : "Четыре измерения сохранены. Проверьте результаты и нажмите Начать регулировку.")
        }
    }

    private var carDiagram: some View {
        VStack(spacing: 12) {
            Text(german ? "VORDERACHSE" : "ПЕРЕДНЯЯ ОСЬ")
                .font(.caption.bold()).foregroundStyle(.cyan.opacity(0.8))
            HStack(alignment: .center, spacing: 5) {
                wheelTile(.frontLeft)
                axleReadout(front: true)
                wheelTile(.frontRight)
            }
            ZStack {
                RoundedRectangle(cornerRadius: 38)
                    .fill(LinearGradient(colors: [Color(red: 0.06, green: 0.22, blue: 0.36), Color(red: 0.01, green: 0.06, blue: 0.12)], startPoint: .leading, endPoint: .trailing))
                    .frame(width: 145, height: 225)
                    .overlay(RoundedRectangle(cornerRadius: 38).stroke(.cyan.opacity(0.6), lineWidth: 2))
                VStack(spacing: 8) {
                    RoundedRectangle(cornerRadius: 15).fill(.black.opacity(0.55))
                        .frame(width: 112, height: 54)
                    centerAction
                    RoundedRectangle(cornerRadius: 12).fill(.black.opacity(0.5))
                        .frame(width: 112, height: 43)
                }
            }
            .frame(height: 235)
            HStack(alignment: .center, spacing: 5) {
                wheelTile(.rearLeft)
                axleReadout(front: false)
                wheelTile(.rearRight)
            }
            Text(german ? "HINTERACHSE" : "ЗАДНЯЯ ОСЬ")
                .font(.caption.bold()).foregroundStyle(.cyan.opacity(0.8))
        }
        .padding(10)
        .background(RoundedRectangle(cornerRadius: 18).fill(Color(red: 0.015, green: 0.07, blue: 0.13)))
        .overlay(RoundedRectangle(cornerRadius: 18).stroke(.cyan.opacity(0.35)))
    }

    // Only a verified vehicle-specific reference can classify a wheel as good or bad.
    private func wheelColor(_ item: Wheel, reading: Double?) -> Color {
        guard let reading else { return .white }
        guard let reference = AlignmentDemoCatalog.verifiedReference(for: selectedVehicle, vin: nil) else { return .cyan }
        let range: ClosedRange<Double> = {
            switch (mode, item) {
            case (.camber, .frontLeft), (.camber, .frontRight): return reference.frontCamberDegrees
            case (.camber, .rearLeft), (.camber, .rearRight): return reference.rearCamberDegrees
            case (.toe, .frontLeft), (.toe, .frontRight): return reference.frontToeDegrees
            case (.toe, .rearLeft), (.toe, .rearRight): return reference.rearToeDegrees
            }
        }()
        return range.contains(reading) ? .green : .red
    }

    private func axleReadout(front: Bool) -> some View {
        let left: Wheel = front ? .frontLeft : .rearLeft
        let right: Wheel = front ? .frontRight : .rearRight
        let readings = mode == .camber ? model.saved : model.toeSaved
        let sum: Double? = {
            guard let a = readings[left], let b = readings[right] else { return nil }
            return mode == .toe ? a + b : a - b
        }()
        return VStack(spacing: 8) {
            Text(mode == .toe ? (german ? "Summe (Test)" : "Сумма (тест)") : (german ? "Links − rechts" : "Левое − правое"))
                .font(.system(size: 11)).foregroundStyle(.secondary)
            Text(sum.map(formatted) ?? "—")
                .font(.system(size: 17, weight: .bold, design: .rounded))
                .foregroundStyle(.cyan)
                .monospacedDigit()
            Capsule().fill(Color.gray.opacity(0.5)).frame(height: 5)
                .overlay(Circle().fill(.green).frame(width: 10, height: 10))
            Text(mode == .toe ? (german ? "Spur TEST" : "Схождение ТЕСТ") :
                    (german ? "Sturzwerte" : "Развал"))
                .font(.system(size: 9)).foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .frame(maxWidth: .infinity)
    }

    private func wheelTile(_ item: Wheel) -> some View {
        let reading = mode == .camber ? model.saved[item] : model.toeSaved[item]
        return VStack(spacing: 7) {
            Text(item.label(german: german))
                .font(.system(size: 10, weight: .semibold))
                .multilineTextAlignment(.center)
                .lineLimit(2)
                .frame(height: 28)
            Text(reading.map(formatted) ?? (activeWheel == item ? (model.liveAngle.map(formatted) ?? "—") : "—"))
                .font(.system(size: 16, weight: .bold, design: .rounded))
                .foregroundStyle(wheelColor(item, reading: reading))
                .monospacedDigit()
            ZStack {
                RoundedRectangle(cornerRadius: 15)
                    .fill(LinearGradient(colors: [.gray.opacity(0.85), .black, .gray.opacity(0.65)], startPoint: .leading, endPoint: .trailing))
                HStack(spacing: 5) {
                    ForEach(0..<4, id: \.self) { _ in
                        Capsule().fill(Color.gray.opacity(0.65)).frame(width: 2)
                    }
                }
                VStack {
                    ForEach(0..<7, id: \.self) { _ in
                        Rectangle().fill(Color.black.opacity(0.65)).frame(height: 2)
                        Spacer(minLength: 0)
                    }
                }.padding(5)
            }
            .frame(width: 56, height: 100)
            .overlay(RoundedRectangle(cornerRadius: 15).stroke(reading != nil ? wheelColor(item, reading: reading) : .gray, lineWidth: 2))
            if reading != nil {
                Label(german ? "Fixiert" : "Сохранено", systemImage: "lock.fill")
                    .font(.system(size: 9)).foregroundStyle(.green)
                Button(german ? "Wiederholen" : "Повторить") {
                    wheel = item
                    activeWheel = nil
                    model.unlock(item)
                }.font(.system(size: 10)).buttonStyle(.bordered)
            } else if activeWheel == item {
                Button(german ? "Speichern" : "Сохранить") {
                    model.save(item)
                    if (mode == .camber ? model.saved[item] : model.toeSaved[item]) != nil {
                        activeWheel = nil
                    }
                }
                .font(.system(size: 10)).buttonStyle(.borderedProminent)
                .disabled(!model.running || !model.stable || !model.correctPosition || model.angle == nil || !model.calibrated)
            } else {
                Button(german ? "Messung" : "Измерить") {
                    wheel = item
                    activeWheel = item
                    model.resetSampling()
                    if !model.running { model.start() }
                }.font(.system(size: 10)).buttonStyle(.bordered)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(5)
        .background(RoundedRectangle(cornerRadius: 12).fill(Color.white.opacity(wheel == item ? 0.12 : 0.04)))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(wheel == item ? .cyan : .gray.opacity(0.25)))

    }

    private var levelView: some View {
        VStack(spacing: 12) {
            ZStack {
                Circle().stroke(.secondary, lineWidth: 5)
                Circle().stroke(.blue.opacity(0.6), lineWidth: 2).padding(16)
                Rectangle().fill(.secondary.opacity(0.6)).frame(width: 2, height: 130)
                Rectangle().fill(.secondary.opacity(0.6)).frame(width: 130, height: 2)
                Capsule().fill(.blue).frame(width: 8, height: 115)
                    .rotationEffect(.degrees(model.liveAngle ?? 0))
                Circle().fill(.blue).frame(width: 15, height: 15)
            }
            .frame(width: 150, height: 150)
            Text(german ? "Live-Neigung (Test)" : "Живой уровень (тест)")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 8)
    }

    private var status: String {
        if model.unavailable {
            return german ? "Bewegungssensor nicht verfügbar." : "Датчик движения недоступен."
        }
        if !model.running {
            return german ? "Messung angehalten." : "Замер остановлен."
        }
        if model.liveAngle == nil {
            return german ? "Warte auf Sensordaten…" : "Ожидание данных датчика…"
        }
        if !model.correctPosition {
            return german ? "Live-Wert aktiv. Für Speichern iPhone senkrecht ausrichten." : "Уровень работает. Для записи поставьте iPhone вертикально."
        }
        if !model.stable {
            return german ? "iPhone ruhig halten." : "Удерживайте iPhone неподвижно."
        }
        return model.calibrated
            ? (german ? "Stabil · bereit zum Speichern" : "Показание устойчиво, можно записать")
            : (german ? "Stabil · Nullpunkt setzen" : "Показание устойчиво, установите ноль")
    }

    private func formatted(_ degrees: Double) -> String {
        let minutes = Int((abs(degrees) * 60).rounded())
        let sign = minutes == 0 ? "" : (degrees < 0 ? "−" : "+")
        return "\(sign)\(minutes / 60)° \(minutes % 60)′"
    }
}

@main
struct AUTOKLINIK3DApp: App {
    var body: some Scene {
        WindowGroup { ContentView() }
    }
}
