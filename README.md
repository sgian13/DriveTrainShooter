# DriveTrainShooter

FTC TeleOp για mecanum drivetrain, intake, δύο ανεξάρτητους shooters και τέσσερα servos. Java, FTC SDK και FTCLib.

> **Σημείωση δομής:** Τα αρχεία Java βρίσκονται προς το παρόν στη ρίζα του repository. Για χρήση στο Android Studio, τοποθέτησε κάθε αρχείο στο `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` και κράτησε τη δήλωση package `org.firstinspires.ftc.teamcode`.

## OpMode και βοηθητικά αρχεία

- Κύριο πρόγραμμα: [DriveTrainServoShooterNEW.java](DriveTrainServoShooterNEW.java)
- Ανίχνευση νέου πατήματος κουμπιού: [Edge.java](Edge.java)
- Έλεγχος shooter / PID: [Shooter.java](Shooter.java)

## Χειριστήρια (gamepad1)

| Κουμπί | Λειτουργία |
|---|---|
| Αριστερό stick | Κίνηση μπρος/πίσω και strafe |
| Δεξί stick X | Στροφή |
| **LB / RB** | Αριστερός / δεξιός shooter ON/OFF |
| **X** | Intake ON/OFF |
| **B** | Slow mode ON/OFF |
| **Y** | Field-centric / robot-centric |
| **LS** | Heading hold ON/OFF στη νεότερη έκδοση |
| **START** | Reset yaw ή επανενεργοποίηση μετά το kill switch στη νεότερη έκδοση |
| **BACK** | Kill switch στη νεότερη έκδοση |
| D-Pad ↑ / ↓ / → / ← | servo1 / servo2 / servo3 / servo4 |

## Hardware configuration

Ρύθμισε τα ονόματα ακριβώς όπως παρακάτω στο Robot Configuration:

| Όνομα | Συσκευή |
|---|---|
| `leftFMotor` | Μπροστινό αριστερό drive motor |
| `rightFMotor` | Μπροστινό δεξί drive motor |
| `leftBMotor` | Πίσω αριστερό drive motor |
| `rightBMotor` | Πίσω δεξί drive motor |
| `intakeMotor` | Intake motor με encoder |
| `shooterLeft` | Αριστερός shooter |
| `shooterRight` | Δεξιός shooter |
| `servo1`–`servo4` | Servos |
| `imu` | IMU του Control Hub |

## Εγκατάσταση

1. Άνοιξε το FTC Robot Controller project στο Android Studio.
2. Πρόσθεσε το FTCLib σύμφωνα με τις [επίσημες οδηγίες εγκατάστασης](https://docs.ftclib.org/ftclib/installation).
3. Αντέγραψε τα DriveTrainServoShooterNEW.java, Edge.java και Shooter.java στον φάκελο `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
4. Κάνε Gradle Sync και Build.
5. Κάνε deploy στο Robot Controller και επίλεξε το αντίστοιχο TeleOp στο Driver Station.

## Έλεγχοι πριν από τη χρήση

- Δοκίμασε τις κατευθύνσεις των drive motors με τους τροχούς σηκωμένους από το έδαφος.
- Επιβεβαίωσε τις κατευθύνσεις των shooters και ότι οι μηχανισμοί είναι ελεύθεροι.
- Έλεγξε τον προσανατολισμό του Control Hub. Οι τιμές Logo UP / USB RIGHT ισχύουν μόνο αν έτσι είναι τοποθετημένο.
- Έλεγξε τις θέσεις servo REST/ACTIVE ώστε να μην πιέζουν μηχανικά τα εξαρτήματα.
- Οι στόχοι RPM και οι ρυθμίσεις PID πρέπει να δοκιμαστούν με ασφάλεια με τα πραγματικά μοτέρ και την τροφοδοσία.

## Αρχεία

- [CHANGELOG.md](CHANGELOG.md)
- [LICENSE](LICENSE)

## Άδεια

MIT — δες το [LICENSE](LICENSE).
