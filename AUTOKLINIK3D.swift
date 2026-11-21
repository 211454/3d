import CoreMotion
import Foundation
import SwiftUI

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
    @Published private(set) var saved: [Wheel: Double] = [:]
    @Published private(set) var toeSaved: [Wheel: Double] = [:]
    @Published private(set) var mode: MeasurementMode = .camber
    private var toeReference: Double?
    private var toeZero = 0.0
    var frontDifference: Double? { guard let l = saved[.frontLeft], let r = saved[.frontRight] else { return nil }; return l - r }
    var rearDifference: Double? { guard let l = saved[.rearLeft], let r = saved[.rearRight] else { return nil }; return l - r }

    private let motion = CMMotionManager()
    private var timer: Timer?
    private var samples: [Double] = []
    private var lastTimestamp: TimeInterval?
    private var zero = 0.0

    func setMode(_ newMode: MeasurementMode) {
        mode = newMode
        calibrated = newMode == .camber ? calibrated : toeReference != nil
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
        angle = nil
    }

    func calibrate() {
        guard stable, let angle else { return }
        if mode == .camber {
            zero += angle
            saved.removeAll()
        } else {
            guard let heading = motion.deviceMotion?.attitude.yaw else { return }
            toeReference = heading
            toeZero = heading
            toeSaved.removeAll()
        }
        calibrated = true
        resetSampling()
    }

    func unlock(_ wheel: Wheel) {
        if mode == .camber { saved.removeValue(forKey: wheel) }
        else { toeSaved.removeValue(forKey: wheel) }
        resetSampling()
    }

    func save(_ wheel: Wheel) {
        guard (mode == .camber ? saved[wheel] : toeSaved[wheel]) == nil else { return }

        guard running, calibrated, correctPosition, let angle, angle.isFinite else { return }
        if mode == .camber { saved[wheel] = angle } else { toeSaved[wheel] = angle }
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
            : (sample.attitude.yaw - toeZero) * 180 / .pi
        guard raw.isFinite else { return }
        let offset = mode == .camber ? zero : 0
        let measured = mode == .camber ? raw - offset : atan2(sin(raw * .pi / 180), cos(raw * .pi / 180)) * 180 / .pi
        liveAngle = measured
        // Portrait, top edge up, screen facing away from the vehicle.
        correctPosition = hypot(gravity.x, gravity.y) > 0.75
        let rotation = sample.rotationRate
        let acceleration = sample.userAcceleration
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
    @State private var activeWheel: Wheel?

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 16) {
                    Picker(german ? "Sprache" : "Язык", selection: $german) {
                        Text("RU").tag(false)
                        Text("DE").tag(true)
                    }.pickerStyle(.segmented)
                    Picker(german ? "Fahrzeug (Demo)" : "Автомобиль (тест)", selection: $selectedVehicle) {
                        ForEach(AlignmentDemoCatalog.models, id: \.self) { Text($0).tag($0) }
                    }
                    Text(german ? "Keine geprüften Herstellervorgaben hinterlegt" : "Заводские нормативы не подключены — только тест")
                        .font(.caption).foregroundStyle(.orange)
                    Picker(german ? "Messung" : "Измерение", selection: $mode) {
                        ForEach(MeasurementMode.allCases) { item in Text(item.label(german)).tag(item) }
                    }.pickerStyle(.segmented)
                    Text(model.liveAngle.map(formatted) ?? "—")
                        .font(.system(size: 38, weight: .bold, design: .rounded))
                        .monospacedDigit()
                    Text(status).font(.caption).foregroundStyle(.secondary)
                    VStack(alignment: .leading, spacing: 8) {
                        Text(german ? "Rolltest (experimentell)" : "Тестовая прокатка").font(.headline)
                        Text(rolling.phase == 1 ? (german ? "Erste Position stabil halten" : "Держи телефон неподвижно для первого замера") :
                             rolling.phase == 2 ? (german ? "Fahrzeug langsam rollen und anhalten" : "Медленно прокати машину и останови") :
                             rolling.phase == 3 ? (german ? "Zwei Positionen gespeichert" : "Два положения записаны") :
                             (german ? "Bereit" : "Готов к тесту"))
                        Button(german ? "Rolltest starten" : "Начать прокатку") { rolling.start() }
                            .buttonStyle(.borderedProminent)
                        if let first = rolling.first { Text("A: \\(formatted(first))") }
                        if let second = rolling.second { Text("B: \\(formatted(second))") }
                        if let difference = rolling.difference { Text((german ? "Differenz: " : "Разница: ") + formatted(difference)) }
                        Text(german ? "Nur Sensortest, keine validierte Achsvermessung." : "Только тест датчика, не точное измерение развала или схождения.")
                            .font(.caption).foregroundStyle(.secondary)
                    }.padding(12)
                    carDiagram
                    Text(german ? "Farben: Türkis = gespeichert, keine OEM-Bewertung. Grün/Rot erst mit geprüften Sollwerten." : "Цвета: голубой — замер сохранён, оценка по заводу недоступна. Зелёный/красный — только с проверенными нормативами.")
                        .font(.caption).foregroundStyle(.secondary)
                    HStack {
                        Button(model.running ? (german ? "Stoppen" : "Остановить") : (german ? "Starten" : "Начать")) {
                            model.running ? model.stop() : model.start()
                        }
                        Spacer()
                        Button(german ? "Nullpunkt" : "Установить ноль") { model.calibrate() }
                            .disabled(!model.stable)
                    }.buttonStyle(.borderedProminent)
                    Text(german
                         ? "Testmessung: Spur ist nicht für eine professionelle Achsvermessung validiert."
                         : "Тестовый режим: точность схождения для профессионального сход-развала не подтверждена.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
                .padding()
            }
            .navigationTitle("AUTOKLINIK 3D")
        .preferredColorScheme(.dark)
        }
        .onChange(of: wheel) { _ in model.resetSampling() }
        .onChange(of: mode) { value in activeWheel = nil; model.setMode(value) }
        .onChange(of: scenePhase) { phase in
            if phase != .active { model.stop(); rolling.stop() }
        }
    }

    private var carDiagram: some View {
        VStack(spacing: 12) {
            Text(german ? "VORDERACHSE" : "ПЕРЕДНЯЯ ОСЬ")
                .font(.caption.bold()).foregroundStyle(.secondary)
            HStack(alignment: .center, spacing: 10) {
                wheelTile(.frontLeft)
                axleReadout(front: true)
                wheelTile(.frontRight)
            }
            Image(systemName: "car.top.door.front.left.open")
                .font(.system(size: 75, weight: .ultraLight))
                .foregroundStyle(.cyan.opacity(0.75))
                .frame(height: 110)
            HStack(alignment: .center, spacing: 10) {
                wheelTile(.rearLeft)
                axleReadout(front: false)
                wheelTile(.rearRight)
            }
            Text(german ? "HINTERACHSE" : "ЗАДНЯЯ ОСЬ")
                .font(.caption.bold()).foregroundStyle(.secondary)
        }
        .padding(10)
        .background(RoundedRectangle(cornerRadius: 18).fill(Color.black))
        .overlay(RoundedRectangle(cornerRadius: 18).stroke(.gray.opacity(0.35)))
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
        .onTapGesture { wheel = item }
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
