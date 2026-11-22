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


// Passat B6 (Typ 3C), 2008: reference values by suspension variant.
// Source: Volkswagen Passat B6 2005–2010 wheel-alignment specification tables.
// Reference only until the vehicle's PR code and the measurement method are verified.
// Toe is an AXLE TOTAL; the half displayed by a wheel is an indicative share,
// not an OEM individual-wheel specification.
enum PassatB6Suspension: String, CaseIterable, Identifiable {
    case standard, sport, sport18
    var id: String { rawValue }

    var prCodes: String {
        switch self {
        case .standard: return "G02 / G07 / G11 / G12 / G15"
        case .sport: return "G03 / G08"
        case .sport18: return "G23 / G28"
        }
    }

    func title(german: Bool) -> String {
        switch self {
        case .standard: return german ? "Serie" : "Стандартная"
        case .sport: return german ? "Sport" : "Спортивная"
        case .sport18: return german ? "Sport 18 Zoll" : "Спорт, 18 дюймов"
        }
    }

    var frontCamberDegrees: Double {
        switch self {
        case .standard: return -30.0 / 60.0
        case .sport, .sport18: return -41.0 / 60.0
        }
    }
    var rearCamberDegrees: Double {
        switch self {
        case .standard, .sport: return -(1.0 + 20.0 / 60.0)
        case .sport18: return -(1.0 + 45.0 / 60.0)
        }
    }
}

enum PassatB6Specs {
    static let camberToleranceDegrees = 30.0 / 60.0
    static let toeAxleTotalDegrees = 10.0 / 60.0
    static let toeAxleToleranceDegrees = 10.0 / 60.0

    static func target(for wheel: Wheel, mode: MeasurementMode, suspension: PassatB6Suspension) -> Double {
        if mode == .toe { return toeAxleTotalDegrees / 2.0 }
        switch wheel {
        case .frontLeft, .frontRight: return suspension.frontCamberDegrees
        case .rearLeft, .rearRight: return suspension.rearCamberDegrees
        }
    }
}
