import AVFoundation
import CoreMotion
import Foundation
import SwiftUI
import Vision
import UIKit

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
    private var toeZeroPending = false
    var frontDifference: Double? { guard let l = saved[.frontLeft], let r = saved[.frontRight] else { return nil }; return l - r }
    var rearDifference: Double? { guard let l = saved[.rearLeft], let r = saved[.rearRight] else { return nil }; return l - r }

    private let motion = CMMotionManager()
    private var timer: Timer?
    private var samples: [Double] = []
    private var lastTimestamp: TimeInterval?
    private var zero = 0.0
    private var camberCalibrated = false
    private let storageKey = "autoklinik3d.measurements.v1"

    init() { /* Each app launch starts a fresh session. */ }

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

    func newSession() {
        saved.removeAll(); toeSaved.removeAll(); zero = 0; toeZero = 0
        toeReference = nil; toeZeroPending = false; camberCalibrated = false; calibrated = false
        resetSampling(); persistMeasurements()
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
        if mode == .toe && toeReference == nil { toeZeroPending = true }
        motion.deviceMotionUpdateInterval = 0.05
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical)
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
        // Capture one session-wide yaw zero only after the phone is flat and still.
        // Never recalibrate when moving to another wheel.
        if mode == .toe && toeReference == nil && toeZeroPending && gravity.z < -0.85 {
            let rate = sample.rotationRate
            let acceleration = sample.userAcceleration
            if hypot(hypot(rate.x, rate.y), rate.z) < 0.08 &&
               hypot(hypot(acceleration.x, acceleration.y), acceleration.z) < 0.08 {
                toeZero = sample.attitude.yaw
                toeReference = toeZero
                toeZeroPending = false
                calibrated = true
            }
        }
        let raw = mode == .camber
            ? atan2(gravity.x, -gravity.y) * 180 / .pi
            : -atan2(sin(sample.attitude.yaw - toeZero), cos(sample.attitude.yaw - toeZero)) * 180 / .pi
        guard raw.isFinite else { return }
        let offset = mode == .camber ? zero : 0
        let measured = mode == .camber ? raw - offset : raw
        liveAngle = mode == .toe && toeReference == nil ? nil : measured
        // Camber: upright phone. Toe: phone flat, screen facing up.
        correctPosition = mode == .camber ? hypot(gravity.x, gravity.y) > 0.75 : gravity.z < -0.85
        let rotation = sample.rotationRate
        let acceleration = sample.userAcceleration
        movementDetected = hypot(hypot(rotation.x, rotation.y), rotation.z) >= 0.15 || hypot(hypot(acceleration.x, acceleration.y), acceleration.z) >= 0.12
        guard correctPosition, (mode != .toe || toeReference != nil),
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


struct ContentView: View {
    @StateObject private var rolling = RollingCapture()
    @StateObject private var model = AlignmentModel()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("alignmentGerman") private var german = false
    @State private var wheel = Wheel.frontLeft
    @State private var mode = MeasurementMode.camber
    @State private var selectedVehicle = "VW Golf 7"
    @State private var documentConfirmed = false
    @State private var documentSummary = ""
    @State private var activeWheel: Wheel?
    @State private var guided = false
    @State private var adjusting = false
    @State private var completed = false
    @State private var awaitingMovement = false
    @State private var speech = AVSpeechSynthesizer()
    @State private var zeroConfirmed = false
    @State private var measurementStarted = false
    @State private var wheelPlacementArmed = true
    @State private var zeroReference: [Wheel: Double] = [:]
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
                    NavigationLink(german ? "Fahrzeugschein fotografieren" : "Фото техпаспорта · подбор автомобиля") { FahrzeugscheinScannerScreen(onConfirm: { summary in
                        documentSummary = summary
                        documentConfirmed = true
                    }) }
                    if documentConfirmed { Text(documentSummary).font(.caption).foregroundStyle(.green) }
                    else { Text(german ? "Schritt 1: Dokument prüfen, danach Sensortest" : "Этап 1: фото и проверка данных, затем тест измерений").font(.caption).foregroundStyle(.orange) }
                        .font(.callout.bold())
                    if documentConfirmed {
                    Picker(german ? "Fahrzeug" : "Автомобиль", selection: $selectedVehicle) {
                        ForEach(AlignmentDemoCatalog.models, id: \.self) { Text($0).tag($0) }
                    }
                    Picker(german ? "Messung" : "Измерение", selection: $mode) {
                        ForEach(MeasurementMode.allCases) { item in Text(item.label(german)).tag(item) }
                    }.pickerStyle(.segmented)
                    Text(german ? "Nur Sensortest — Spurwerte nicht validiert" : "Тест датчиков: точность схождения не подтверждена")
                        .font(.caption).foregroundStyle(.orange)
                    Text(guided ? (activeWheel?.label(german: german) ?? "") : completed ? (german ? "Messungen abgeschlossen" : "Проверка завершена") : adjusting ? (german ? "Einstellmodus" : "Режим регулировки") : (german ? "Bereit zur Prüfung" : "Готово к проверке"))
                        .font(.headline).foregroundStyle(.white)
                    Text(model.liveAngle.map(formatted) ?? "—")
                        .font(.system(size: 32, weight: .bold, design: .rounded))
                        .foregroundStyle(.cyan).monospacedDigit()
                    carDiagram
                    }
                    if guided {
                        Text(awaitingMovement ? (german ? "Telefon zum nächsten Rad bewegen" : "Перенесите телефон на следующее колесо") : (german ? "Vier Sekunden stabil halten" : "Держите неподвижно четыре секунды"))
                            .font(.callout)
                        Text(String(format: "%.1f / 4.0 s", min(model.stableSeconds, 4))).monospacedDigit()
                    }
                    Text(status).font(.caption).foregroundStyle(.secondary)
                    Text(german ? "Türkis: gespeichert; keine Bewertung ohne geprüfte Sollwerte." : "Голубой: сохранено. Без заводских допусков оценка недоступна.")
                        .font(.caption2).foregroundStyle(.secondary)
                }.padding(14)
            }
            .background(
                ZStack {
                    LinearGradient(colors: [Color(red: 0.005, green: 0.018, blue: 0.04), Color(red: 0.015, green: 0.11, blue: 0.20), .black], startPoint: .top, endPoint: .bottom)
                    GeometryReader { geo in
                        Path { path in
                            let horizon = geo.size.height * 0.55
                            for index in 0...14 {
                                let x = geo.size.width * CGFloat(index) / 14
                                path.move(to: CGPoint(x: geo.size.width / 2, y: horizon))
                                path.addLine(to: CGPoint(x: x, y: geo.size.height))
                            }
                            for index in 0...9 {
                                let fraction = CGFloat(index) / 9
                                let y = horizon + (geo.size.height - horizon) * fraction * fraction
                                path.move(to: CGPoint(x: 0, y: y))
                                path.addLine(to: CGPoint(x: geo.size.width, y: y))
                            }
                        }
                        .stroke(Color.cyan.opacity(0.13), lineWidth: 0.7)
                        LinearGradient(colors: [.cyan.opacity(0.10), .clear], startPoint: .top, endPoint: .bottom)
                    }
                }
                .ignoresSafeArea()
            )
            .navigationBarHidden(true)
            .preferredColorScheme(.dark)
        }
        .onAppear {
            if documentConfirmed && !guided && !completed && !adjusting { beginGuided() }
        }
        .onChange(of: documentConfirmed) { confirmed in if confirmed { beginGuided() } }
        .onChange(of: wheel) { _ in model.resetSampling() }
        .onChange(of: model.stableSeconds) { _ in advanceGuidedIfReady() }
        .onChange(of: model.movementDetected) { moving in
            if guided && awaitingMovement && moving {
                awaitingMovement = false
                wheelPlacementArmed = true
                model.resetSampling()
            }
        }
        .onChange(of: mode) { value in model.setMode(value) }
        .onChange(of: scenePhase) { phase in
            if phase != .active { model.stop(); rolling.stop() }
        }
    }

    private let testTolerance = 1.0 // TEST ONLY: not a factory specification

    private var adjustmentTarget: Wheel? {
        wheelOrder.filter { abs((mode == .toe ? model.toeSaved : model.saved)[$0] ?? 0) > testTolerance }
            .max { abs((mode == .toe ? model.toeSaved : model.saved)[$0] ?? 0) < abs((mode == .toe ? model.toeSaved : model.saved)[$1] ?? 0) }
    }

    private func beginAdjustment() {
        guard completed else { return }
        adjusting = true
        completed = false
        activeWheel = adjustmentTarget
        if let target = activeWheel {
            wheel = target
            model.resetSampling()
            if !model.running { model.start() }
            speak((german ? "Telefon am Rad befestigen und Einstellung beginnen: " :
                "Установите телефон на колесо и начните регулировку: ") + target.label(german: german))
        } else {
            speak(german ? "Alle Messwerte innerhalb der Testtoleranz." :
                "Все измерения в пределах тестового допуска.")
        }
    }

    private var centerAction: some View {
        Text(adjusting ? (german ? "EINSTELLUNG" : "РЕГУЛИРОВКА") :
             (german ? "AUTOMATISCHE MESSUNG" : "АВТОМАТИЧЕСКОЕ ИЗМЕРЕНИЕ"))
            .font(.system(size: 11, weight: .bold))
            .foregroundStyle(.cyan)
            .multilineTextAlignment(.center)
    }

    private func speak(_ message: String) {
        do {
            let session = AVAudioSession.sharedInstance()
            try session.setCategory(.playback, mode: .spokenAudio, options: [.duckOthers])
            try session.setActive(true)
        } catch {
            print("Speech audio session: \(error.localizedDescription)")
        }
        speech.stopSpeaking(at: .immediate)
        let utterance = AVSpeechUtterance(string: message)
        utterance.voice = AVSpeechSynthesisVoice(language: german ? "de-DE" : "ru-RU")
        utterance.volume = 1.0
        speech.speak(utterance)
    }

    private func beginGuided() {
        guided = true
        completed = false
        adjusting = false
        awaitingMovement = false
        zeroConfirmed = false
        measurementStarted = false
        wheelPlacementArmed = true
        zeroReference.removeAll()
        model.newSession()
        mode = .toe
        model.setMode(.toe)
        activeWheel = .frontLeft
        if activeWheel == nil { guided = false; completed = true; return }
        wheel = activeWheel ?? .frontLeft
        model.resetSampling()
        if !model.running { model.start() }
        speak(german ? "Telefon am linken Vorderrad befestigen. Nullpunkt prüfen." : "Установите телефон на переднее левое колесо. Держите неподвижно четыре секунды.")
    }

    private func advanceGuidedIfReady() {
        guard guided, !awaitingMovement, wheelPlacementArmed,
              model.stableSeconds >= 4, let current = activeWheel,
              let rawAngle = model.angle else { return }
        wheelPlacementArmed = false
        // A four-second stationary window is a stability check, not a
        // physical wheel-angle zero: never subtract the wheel's own tilt.
        if current == .frontLeft { zeroReference[current] = 0 }
        zeroConfirmed = true
        measurementStarted = true
        model.save(current, guidedCapture: true)
        guard (mode == .toe ? model.toeSaved[current] : model.saved[current]) != nil else {
            wheelPlacementArmed = true
            return
        }
        if let next = wheelOrder.first(where: { (mode == .toe ? model.toeSaved[$0] : model.saved[$0]) == nil }) {
            activeWheel = next
            wheel = next
            awaitingMovement = true
            zeroConfirmed = false
            measurementStarted = false
            model.resetSampling()
            speak((german ? "Gespeichert. Telefon am nächsten Rad befestigen: " :
                "Измерение сохранено. Снимите телефон и установите на следующее колесо: ") +
                next.label(german: german))
        } else {
            guided = false
            completed = true
            activeWheel = nil
            beginAdjustment()
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
            Text(adjusting && activeWheel == item ? (model.liveAngle.map(formatted) ?? "—") : (reading.map(formatted) ?? (activeWheel == item ? (model.liveAngle.map(formatted) ?? "—") : "—")))
                .font(.system(size: 16, weight: .bold, design: .rounded))
                .foregroundStyle(adjusting && activeWheel == item ? ((model.liveAngle.map { abs($0) <= testTolerance } ?? false) ? Color.green : Color.red) : wheelColor(item, reading: reading))
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
            if adjusting && activeWheel == item {
                Text((german ? "Start: " : "Было: ") + (reading.map(formatted) ?? "—"))
                    .font(.system(size: 9)).foregroundStyle(.orange)
            } else if reading != nil {
                Label(german ? "Fixiert" : "Сохранено", systemImage: "lock.fill")
                    .font(.system(size: 9)).foregroundStyle(.green)
            } else if activeWheel == item {
                Text(german ? "Automatisch" : "Автоматически")
                    .font(.system(size: 9)).foregroundStyle(.cyan)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(5)
        .background(RoundedRectangle(cornerRadius: 12).fill(Color.white.opacity(wheel == item ? 0.12 : 0.04)))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(adjusting && activeWheel == item ? .red : (wheel == item ? .cyan : .gray.opacity(0.25))))

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


// MARK: - Fahrzeugschein photo identification (offline; unverified matching is prohibited)
struct FahrzeugscheinScannerScreen: View {
    let onConfirm: (String) -> Void
    @State private var showingCamera = false
    @State private var fields: [String: String] = [:]
    @State private var processing = false
    @State private var errorText: String?
    @State private var manuallyConfirmed = false
    @State private var suspensionCode = ""
    private var factoryMatch: FactoryAlignmentEntry? {
        FactoryAlignmentDatabase.find(fields: fields, suspensionCode: suspensionCode)
    }
    var body: some View {
        Form {
            Section("Zulassungsbescheinigung Teil I") {
                Button("Сфотографировать документ") { showingCamera = true }
                    .disabled(!UIImagePickerController.isSourceTypeAvailable(.camera))
                if processing { ProgressView("Распознавание…") }
                if let errorText { Text(errorText).foregroundStyle(.red) }
            }
            if !fields.isEmpty {
                Section("Распознано — проверьте по документу") {
                    ForEach(["E", "2.1", "2.2", "B", "D.1", "D.2", "D.3", "K", "P.1", "P.2"], id: \.self) { field in
                        HStack {
                            Text(field)
                            Spacer()
                            Text(fields[field] ?? "Не найдено").foregroundStyle(.secondary)
                        }
                    }
                }
                Section("Проверка автомобиля") {
                    TextField("Код подвески (например, 1BA)", text: $suspensionCode)
                    if let meaning = FactoryAlignmentDatabase.vagSuspensionCodes[suspensionCode.uppercased()] {
                        Text(meaning).font(.caption).foregroundStyle(.secondary)
                    }
                        .textInputAutocapitalization(.characters)
                    if let match = factoryMatch {
                        Text("Найдены заводские данные: \\(match.make) \\(match.model) — \\(match.suspensionCode)")
                        Text("Источник: \\(match.source)")
                        Text(String(format: "Передний развал: %.2f° ± %.2f°", match.frontCamber, match.frontCamberTolerance))
                        Text(String(format: "Переднее схождение на колесо: %.2f° ± %.2f°", match.frontToePerWheel, match.frontToeTolerance))
                    } else {
                        Text("Проверенных заводских углов для точной модификации и подвески пока нет. Нельзя использовать тестовые углы как заводские.")
                            .foregroundStyle(.orange)
                    }
                    Text("Сверьте VIN, HSN/TSN и подвеску с документом. Без заводских данных измерения остаются только тестом.")
                    Toggle("Я сверил данные с техпаспортом", isOn: $manuallyConfirmed)
                    Button("Перейти к измерениям (тест)") {
                        let summary = [fields["D.1"], fields["D.3"], fields["E"]].compactMap { $0 }.joined(separator: " · ")
                        onConfirm(summary)
                    }.disabled(!manuallyConfirmed || fields["E"] == nil)
                }
            }
        }
        .navigationTitle("Фото · автомобиль")
        .sheet(isPresented: $showingCamera) {
            FahrzeugscheinCamera { image in
                showingCamera = false
                guard let image, let cg = image.cgImage else { return }
                processing = true
                Task.detached(priority: .userInitiated) {
                    let request = VNRecognizeTextRequest()
                    request.recognitionLevel = .accurate
                    request.recognitionLanguages = ["de-DE"]
                    request.usesLanguageCorrection = false
                    do {
                        try VNImageRequestHandler(cgImage: cg).perform([request])
                        let lines = (request.results ?? []).compactMap { $0.topCandidates(1).first?.string }
                        let parsed = FahrzeugscheinFields.parse(lines)
                        await MainActor.run { fields = parsed; manuallyConfirmed = false; processing = false; errorText = nil }
                    } catch {
                        await MainActor.run { processing = false; errorText = error.localizedDescription }
                    }
                }
            }
        }
    }
}

private enum FahrzeugscheinFields {
    static func parse(_ lines: [String]) -> [String: String] {
        var output: [String: String] = [:]
        let keys = ["2.1", "2.2", "D.1", "D.2", "D.3", "E", "B", "K", "P.1", "P.2"]
        for (i, raw) in lines.enumerated() {
            let line = raw.trimmingCharacters(in: .whitespacesAndNewlines)
            for key in keys where output[key] == nil {
                let pattern = "^" + NSRegularExpression.escapedPattern(for: key) + "(?:\\s*[:.\\-]?\\s*)(.*)$"
                guard let regex = try? NSRegularExpression(pattern: pattern, options: .caseInsensitive) else { continue }
                let range = NSRange(line.startIndex..<line.endIndex, in: line)
                guard let match = regex.firstMatch(in: line, range: range),
                      let valueRange = Range(match.range(at: 1), in: line) else { continue }
                let sameLine = String(line[valueRange]).trimmingCharacters(in: .whitespacesAndNewlines)
                let value = sameLine.isEmpty && i + 1 < lines.count ? lines[i + 1] : sameLine
                if !value.isEmpty { output[key] = value }
            }
        }
        // Reject obvious OCR confusion rather than invent an identity.
        if let vin = output["E"], vin.uppercased().range(of: "^[A-HJ-NPR-Z0-9]{17}$", options: .regularExpression) == nil {
            output.removeValue(forKey: "E")
        }
        return output
    }
}

private struct FahrzeugscheinCamera: UIViewControllerRepresentable {
    let onComplete: (UIImage?) -> Void
    func makeCoordinator() -> Coordinator { Coordinator(onComplete: onComplete) }
    func makeUIViewController(context: Context) -> UIImagePickerController {
        let controller = UIImagePickerController()
        controller.sourceType = .camera
        controller.delegate = context.coordinator
        return controller
    }
    func updateUIViewController(_ controller: UIImagePickerController, context: Context) {}
    final class Coordinator: NSObject, UINavigationControllerDelegate, UIImagePickerControllerDelegate {
        let onComplete: (UIImage?) -> Void
        init(onComplete: @escaping (UIImage?) -> Void) { self.onComplete = onComplete }
        func imagePickerController(_ picker: UIImagePickerController, didFinishPickingMediaWithInfo info: [UIImagePickerController.InfoKey: Any]) {
            onComplete(info[.originalImage] as? UIImage)
        }
        func imagePickerControllerDidCancel(_ picker: UIImagePickerController) { onComplete(nil) }
    }
}


// MARK: - Strict factory alignment database matching
// Only records supported by an identifiable source may be added here.
// A VIN and HSN/TSN do not by themselves establish suspension PR codes.
private struct FactoryAlignmentEntry {
    let make: String
    let model: String
    let suspensionCode: String
    let vinPrefix: String
    let hsn: String
    let tsn: String
    let frontCamber: Double
    let frontCamberTolerance: Double
    let frontToePerWheel: Double
    let frontToeTolerance: Double
    let source: String
}

private enum FactoryAlignmentDatabase {
    // Intentionally empty until authoritative variant-specific records and
    // VIN/HSN/TSN mappings are validated. Never fabricate factory limits.
    // Vehicle discovery catalog only. Factory alignment values still require
    // verified records for the exact suspension and vehicle variant.
    static let modelYearThrough = 2020
    static let volkswagenModels = [
        "up!", "Polo", "Golf", "Golf Plus", "Golf Sportsvan",
        "Jetta", "Bora", "Passat", "Arteon", "Scirocco",
        "Beetle", "New Beetle", "Touran", "Sharan", "Tiguan",
        "Touareg", "T-Roc", "T-Cross", "Caddy", "Transporter",
        "Multivan", "Caravelle", "California", "Crafter", "Amarok"
    ]
    static let audiModels = [
        "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8",
        "Q2", "Q3", "Q5", "Q7", "Q8", "TT", "R8"
    ]
    // VAG PR codes identify suspension equipment, not alignment angles.
    // These must be confirmed on the vehicle data sticker/service record.
    static let vagSuspensionCodes: [String: String] = [
        "1BA": "Standard suspension / Serienfahrwerk",
        "1BE": "Sports suspension / Sportfahrwerk",
        "1BD": "Sports suspension variant; verify exact model",
        "1BP": "Suspension variant; verify exact model"
    ]
    static let verified: [FactoryAlignmentEntry] = []

    static func find(fields: [String: String], suspensionCode: String) -> FactoryAlignmentEntry? {
        guard let vin = fields["E"]?.uppercased(),
              vin.range(of: "^[A-HJ-NPR-Z0-9]{17}$", options: .regularExpression) != nil,
              let hsn = fields["2.1"]?.trimmingCharacters(in: .whitespacesAndNewlines),
              let tsn = fields["2.2"]?.trimmingCharacters(in: .whitespacesAndNewlines),
              let make = fields["D.1"]?.uppercased(),
              let model = fields["D.3"]?.uppercased(),
              !suspensionCode.isEmpty else { return nil }
        return verified.first {
            $0.make.uppercased() == make &&
            $0.model.uppercased() == model &&
            $0.suspensionCode.uppercased() == suspensionCode.uppercased() &&
            $0.hsn == hsn && $0.tsn == tsn &&
            vin.hasPrefix($0.vinPrefix.uppercased())
        }
    }
}
