import Foundation

// Test-only data. These values are NOT manufacturer specifications.
struct AlignmentReference: Codable {
    let model: String
    let frontCamberDegrees: ClosedRange<Double>
    let frontToeDegrees: ClosedRange<Double>
    let rearCamberDegrees: ClosedRange<Double>
    let rearToeDegrees: ClosedRange<Double>
    let isDemonstration: Bool
}

enum WheelPosition: String, CaseIterable, Codable {
    case frontLeft, frontRight, rearLeft, rearRight
}

struct WheelReading: Codable {
    let camberDegrees: Double
    let toeDegrees: Double
    let capturedAt: Date
}

struct AlignmentTestSession {
    private(set) var readings: [WheelPosition: WheelReading] = [:]
    var completed: Bool { WheelPosition.allCases.allSatisfy { readings[$0] != nil } }

    mutating func save(_ reading: WheelReading, for wheel: WheelPosition) -> Bool {
        guard readings[wheel] == nil else { return false } // Locked after saving
        readings[wheel] = reading
        return true
    }

    mutating func unlockForRemeasurement(_ wheel: WheelPosition) {
        readings.removeValue(forKey: wheel)
    }
}

enum AlignmentDemoCatalog {
    static let models = [
        "VW Golf 5", "VW Golf 6", "VW Golf 7", "VW Golf 8",
        "VW Passat B6", "VW Passat B7", "VW Passat B8",
        "Audi A3", "Audi A4", "Audi A5", "Audi A6",
        "Mercedes-Benz C-Class", "Mercedes-Benz E-Class"
    ]
    // No fabricated OEM limits: the UI must explicitly show "Demo / no verified OEM data".
    static func verifiedReference(for model: String, vin: String?) -> AlignmentReference? { nil }
}
