import CoreMotion
import Foundation

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
