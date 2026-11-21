import CoreMotion
import Foundation
import SwiftUI

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

    private let motion = CMMotionManager()
    private var timer: Timer?
    private var samples: [Double] = []
    private var lastTimestamp: TimeInterval?
    private var zero = 0.0

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
        zero += angle
        calibrated = true
        saved.removeAll()
        resetSampling()
    }

    func save(_ wheel: Wheel) {
        guard running, stable, calibrated, let angle else { return }
        saved[wheel] = angle
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
        let raw = atan2(gravity.z, hypot(gravity.x, gravity.y)) * 180 / .pi
        guard raw.isFinite else { return }
        liveAngle = raw - zero
        // Portrait, top edge up, screen facing away from the vehicle.
        correctPosition = abs(gravity.y) > 0.75 && abs(gravity.x) < 0.55
        let rotation = sample.rotationRate
        let acceleration = sample.userAcceleration
        guard correctPosition,
              hypot(hypot(rotation.x, rotation.y), rotation.z) < 0.15,
              hypot(hypot(acceleration.x, acceleration.y), acceleration.z) < 0.12 else {
            resetSampling()
            return
        }

        samples.append(raw)
        if samples.count > 20 { samples.removeFirst() }
        let minimum = samples.min() ?? raw
        let maximum = samples.max() ?? raw
        angle = samples.reduce(0, +) / Double(samples.count) - zero
        stable = samples.count == 20 && maximum - minimum <= 0.30
    }
}

struct ContentView: View {
    @StateObject private var model = AlignmentModel()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("alignmentGerman") private var german = false
    @State private var wheel = Wheel.frontLeft

    var body: some View {
        NavigationStack {
            Form {
                Section {
                    Picker(german ? "Sprache" : "Язык", selection: $german) {
                        Text("RU").tag(false)
                        Text("DE").tag(true)
                    }
                    .pickerStyle(.segmented)

                    Text(german
                         ? "iPhone hochkant befestigen, Display nach außen. Den Nullpunkt zuerst mit derselben Halterung an einer senkrechten Referenz setzen."
                         : "Закрепите iPhone вертикально, экраном наружу. Сначала обнулите прибор с тем же креплением на эталонной вертикальной плоскости.")
                    .font(.footnote)
                }

                Section(german ? "Sturz · Testmessung" : "Развал · пробный замер") {
                    Picker(german ? "Rad" : "Колесо", selection: $wheel) {
                        ForEach(Wheel.allCases) { item in
                            Text(item.label(german: german)).tag(item)
                        }
                    }
                    Text(model.liveAngle.map(formatted) ?? "—")
                        .font(.system(size: 44, weight: .bold, design: .rounded))
                        .monospacedDigit()
                        .accessibilityLabel(german ? "Aktueller Winkel" : "Текущий угол")
                        .accessibilityValue(model.angle.map(formatted) ?? "—")
                    levelView
                    Text(status)
                        .font(.footnote)
                        .foregroundStyle(.secondary)

                    Button(model.running
                           ? (german ? "Stoppen" : "Остановить")
                           : (german ? "Starten" : "Начать")) {
                        model.running ? model.stop() : model.start()
                    }
                    Button(german ? "Nullpunkt setzen" : "Установить ноль") {
                        model.calibrate()
                    }
                    .disabled(!model.stable)
                    Button(german ? "Radwert speichern" : "Записать замер колеса") {
                        model.save(wheel)
                    }
                    .disabled(!model.stable || !model.calibrated)
                }

                Section(german ? "Messwerte dieser Sitzung" : "Замеры этой сессии") {
                    ForEach(Wheel.allCases) { item in
                        LabeledContent(item.label(german: german),
                                       value: model.saved[item].map(formatted) ?? "—")
                    }
                    Text(german
                         ? "Die Genauigkeit am Fahrzeug ist ungeprüft. Felgenschlag und Standflächenneigung werden nicht kompensiert. Spur ist noch nicht verfügbar."
                         : "Точность на автомобиле предстоит проверить. Биение диска и уклон площадки не компенсируются. Измерение схождения пока недоступно.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                }
            }
            .navigationTitle("AUTOKLINIK 3D")
        }
        .onChange(of: wheel) { _ in model.resetSampling() }
        .onChange(of: scenePhase) { phase in
            if phase != .active { model.stop() }
        }
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
