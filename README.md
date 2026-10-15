# DriveTrainShooter

TeleOp πρόγραμμα για ρομπότ **FTC (FIRST Tech Challenge)** με mecanum drivetrain, δύο shooters, intake και τέσσερα servos. Γράφτηκε σε Java με το FTC SDK και τη βιβλιοθήκη [FTCLib](https://github.com/FTCLib/FTCLib).

## Χαρακτηριστικά

- Mecanum drive με **Robot Centric** και **Field Centric** (μέσω IMU), εναλλαγή με ένα κουμπί
- **Slow mode** (30%) για ακριβείς κινήσεις
- Δύο shooters με ON/OFF toggle και live telemetry (ticks, RPM)
- Προαιρετικό debug για τον αριστερό shooter: peak RPM, peak ρεύμα, spin-up time
- Intake με ON/OFF toggle και telemetry RPM
- Τέσσερα servos στο D-Pad, με εναλλαγή REST ↔ ACTIVE

## Χειριστήρια (gamepad1)

| Κουμπί | Λειτουργία |
|---|---|
| Αριστερό stick | Κίνηση (μπρος/πίσω/strafe) |
| Δεξί stick (οριζόντια) | Στροφή |
| **LB** | Shooter Left ON/OFF |
| **RB** | Shooter Right ON/OFF |
| **X** | Intake ON/OFF |
| **B** | Slow mode ON/OFF |
| **Y** | Field Centric / Robot Centric (κάνει reset το yaw όταν μπαίνει σε Field Centric) |
| D-Pad ↑ / ↓ / → / ← | servo1 / servo2 / servo3 / servo4 |

## Hardware configuration

Τα ονόματα στο Robot Configuration του Driver Hub πρέπει να είναι ακριβώς τα παρακάτω:

| Όνομα | Τύπος | Περιγραφή |
|---|---|---|
| `leftFMotor` | Motor | Μπροστά αριστερά |
| `rightFMotor` | Motor | Μπροστά δεξιά |
| `leftBMotor` | Motor | Πίσω αριστερά |
| `rightBMotor` | Motor | Πίσω δεξιά |
| `intakeMotor` | Motor (με encoder) | Intake |
| `shooterLeft` | Motor (με encoder) | Αριστερός shooter |
| `shooterRight` | Motor (με encoder) | Δεξιός shooter |
| `servo1` – `servo4` | Servo | Servos του D-Pad |
| `imu` | IMU | Ενσωματωμένο IMU του Control Hub |

## Εγκατάσταση

Απαιτείται FTC SDK **8.1 ή νεότερο** (για το ενιαίο `IMU` interface) και το FTCLib.

1. Κάνε clone ή κατέβασε το [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController).
2. Πρόσθεσε το FTCLib στο project σύμφωνα με τις [οδηγίες εγκατάστασης του FTCLib](https://docs.ftclib.org/ftclib/installation) (dependency στο `build.dependencies.gradle`).
3. Αντίγραψε το αρχείο
   `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DriveTrainServoShooter.java`
   στον αντίστοιχο φάκελο του δικού σου project.
4. Κάνε build και deploy στο Robot Controller.
5. Στο Driver Hub διάλεξε TeleOp **Grobo-DriveTrain_Shooter_Servo**.

## Ρυθμίσεις

Στην αρχή του αρχείου υπάρχουν σταθερές που μπορείς να αλλάξεις:

| Σταθερά | Σημασία |
|---|---|
| `SHOOTER_POWER` | Ισχύς των shooters όταν είναι ON (0–1) |
| `SHOOTER_TICKS_PER_REV`, `INTAKE_TICKS_PER_REV` | Ticks ανά περιστροφή του encoder |
| `SHOOTER_MAX_RPM`, `SPIN_UP_FRACTION` | Όριο για το spin-up time |
| `SHOOTER_DEBUG_TELEMETRY` | `true` για peak RPM, ρεύμα και spin-up στο telemetry |
| `SERVO_REST`, `SERVO_ACTIVE` | Θέσεις των servos |
| `SLOW_DRIVE`, `SLOW_TURN` | Ταχύτητα στο slow mode |

## Έλεγχοι πριν από τον πρώτο αγώνα

- **Κατευθύνσεις των drive motors:** το FTCLib `MecanumDrive` δεν αντιστρέφει μόνο του τα δεξιά motors. Τέστ με σηκωμένο ρομπότ.
- **Κατευθύνσεις των shooters:** τώρα και τα δύο είναι `REVERSE`. Αν είναι τοποθετημένα αντικριστά, το ένα πρέπει να γίνει `FORWARD`.
- **Προσανατολισμός του Control Hub:** το `LogoFacingDirection` / `UsbFacingDirection` πρέπει να ταιριάζει με το πώς είναι στερεωμένο. Αλλιώς το Field Centric θα στρίβει λάθος.
- **Servos:** ρύθμισε τα `SERVO_REST` / `SERVO_ACTIVE` ώστε να μη φτάνουν στο μηχανικό όριο.

## Δομή

```
DriveTrainShooter/
├── TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
│   └── DriveTrainServoShooter.java
├── CHANGELOG.md
├── LICENSE
└── README.md
```

## Άδεια

MIT, δες το αρχείο [LICENSE](LICENSE).
