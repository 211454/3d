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

    func save(_ wheel: Wheel) {
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
            ? atan2(gravity.z, hypot(gravity.x, gravity.y)) * 180 / .pi
            : (sample.attitude.yaw - toeZero) * 180 / .pi
        guard raw.isFinite else { return }
        let offset = mode == .camber ? zero : 0
        let measured = mode == .camber ? raw - offset : atan2(sin(raw * .pi / 180), cos(raw * .pi / 180)) * 180 / .pi
        liveAngle = measured
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

        samples.append(measured)
        if samples.count > 20 { samples.removeFirst() }
        let minimum = samples.min() ?? raw
        let maximum = samples.max() ?? raw
        angle = samples.reduce(0, +) / Double(samples.count)
        stable = samples.count == 20 && maximum - minimum <= 0.30
    }
}

struct ContentView: View {
    @StateObject private var model = AlignmentModel()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("alignmentGerman") private var german = false
    @State private var wheel = Wheel.frontLeft
    @State private var mode = MeasurementMode.camber

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 16) {
                    Picker(german ? "Sprache" : "Язык", selection: $german) {
                        Text("RU").tag(false)
                        Text("DE").tag(true)
                    }.pickerStyle(.segmented)
                    Picker(german ? "Messung" : "Измерение", selection: $mode) {
                        ForEach(MeasurementMode.allCases) { item in Text(item.label(german)).tag(item) }
                    }.pickerStyle(.segmented)
                    Text(model.liveAngle.map(formatted) ?? "—")
                        .font(.system(size: 38, weight: .bold, design: .rounded))
                        .monospacedDigit()
                    Text(status).font(.caption).foregroundStyle(.secondary)
                    carDiagram
                    HStack {
                        Text(german ? "Vorne L−R" : "Разница спереди")
                        Spacer()
                        Text(model.frontDifference.map(formatted) ?? "—")
                    }
                    HStack {
                        Text(german ? "Hinten L−R" : "Разница сзади")
                        Spacer()
                        Text(model.rearDifference.map(formatted) ?? "—")
                    }
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
        }
        .onChange(of: wheel) { _ in model.resetSampling() }
        .onChange(of: mode) { value in model.setMode(value) }
        .onChange(of: scenePhase) { phase in
            if phase != .active { model.stop() }
        }
    }

    private var carDiagram: some View {
        HStack(alignment: .center, spacing: 8) {
            VStack(spacing: 28) {
                wheelTile(.frontLeft)
                wheelTile(.rearLeft)
            }
            VStack(spacing: 0) {
                Text(german ? "VORNE" : "ПЕРЕД").font(.caption2).foregroundStyle(.secondary)
                ZStack {
                    RoundedRectangle(cornerRadius: 25)
                        .fill(LinearGradient(colors: [.gray.opacity(0.45), .blue.opacity(0.15)], startPoint: .top, endPoint: .bottom))
                        .overlay(RoundedRectangle(cornerRadius: 25).stroke(.blue, lineWidth: 2))
                    VStack(spacing: 12) {
                        RoundedRectangle(cornerRadius: 12).fill(.black.opacity(0.35)).frame(height: 44)
                        Spacer()
                        Image(systemName: "car.top.door.front.left.open")
                            .font(.system(size: 32)).foregroundStyle(.blue)
                        Spacer()
                        RoundedRectangle(cornerRadius: 12).fill(.black.opacity(0.35)).frame(height: 35)
                    }.padding(10)
                }
                .frame(maxWidth: 110, minHeight: 290)
                Text(german ? "HINTEN" : "ЗАД").font(.caption2).foregroundStyle(.secondary)
            }
            VStack(spacing: 28) {
                wheelTile(.frontRight)
                wheelTile(.rearRight)
            }
        }
        .frame(maxWidth: .infinity)
    }

    private func wheelTile(_ item: Wheel) -> some View {
        let reading = mode == .camber ? model.saved[item] : model.toeSaved[item]
        return VStack(spacing: 6) {
            Text(item.label(german: german))
                .font(.system(size: 10, weight: .semibold))
                .multilineTextAlignment(.center)
                .lineLimit(2)
                .frame(height: 28)
            Image(systemName: "circle.hexagongrid.fill")
                .font(.system(size: 28))
                .foregroundStyle(wheel == item ? .blue : .secondary)
            Text(reading.map(formatted) ?? "—")
                .font(.system(size: 15, weight: .bold, design: .rounded))
                .monospacedDigit()
            Button(german ? "Speichern" : "Записать") {
                wheel = item
                model.save(item)
            }
            .font(.system(size: 11))
            .buttonStyle(.bordered)
            .disabled(!model.running || !model.correctPosition || model.angle == nil || !model.calibrated)
        }
        .frame(maxWidth: .infinity)
        .padding(6)
        .background(RoundedRectangle(cornerRadius: 12).fill(Color.blue.opacity(wheel == item ? 0.15 : 0.06)))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(.blue.opacity(0.5)))
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
