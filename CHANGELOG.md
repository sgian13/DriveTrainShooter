# Changelog

## [Unreleased]

### Διορθώσεις
- Το αρχείο του OpMode είχε όνομα `DriveTrainServoShooter` χωρίς επέκταση `.java`. Δεν γινόταν compile. Μετονομάστηκε και μπήκε στη σωστή δομή `TeamCode/src/main/java/...`.
- Το σχόλιο στην αρχή έγραφε ότι το slow mode είναι στο **A**, ενώ ο κώδικας το χρησιμοποιεί στο **B**.
- Οι μεταβλητές `lastLB` / `lastRB` ήταν δηλωμένες δίπλα στο λάθος shooter (δούλευε, αλλά μπέρδευε). Διορθώθηκε η σειρά.
- Το `SHOOTER_MAX_RPM` ήταν δηλωμένο μέσα στο loop. Μπήκε στις σταθερές στην αρχή.
- Οι μεταβλητές `shooterLeftPeakRpm`, `shooterLeftPeakCurrent`, `shooterLeftSpinUpTime` υπολογίζονταν αλλά δεν εμφανίζονταν πουθενά. Εμφανίζονται τώρα με `SHOOTER_DEBUG_TELEMETRY = true`. Το ρεύμα διαβάζεται από το hub μόνο όταν αυτό είναι ενεργό.
- Αφαιρέθηκε η αχρησιμοποίητη `shooterLeftStartTicks`.
- Το σχόλιο για το telemetry των servos έλεγε "πραγματική θέση". Το `getPosition()` επιστρέφει την τελευταία εντολή, όχι ανάγνωση αισθητήρα.
- Το heading του IMU διαβαζόταν δύο φορές ανά loop. Διαβάζεται πλέον μία.
- Προστέθηκε `drive.stop()` στο τέλος του OpMode.

### Προσθήκες
- Ο δεξιός shooter έγινε `DcMotorEx` και έχει telemetry RPM.
- Τα hardcoded στοιχεία (power, ticks, θέσεις servo, slow mode) μαζεύτηκαν σε σταθερές στην αρχή του αρχείου.
- README, LICENSE, `.gitignore`, CHANGELOG.

- Ενημερώθηκε το README για τη δομή των αρχείων, τις εξαρτήσεις, τα χειριστήρια και τους ελέγχους ασφαλείας.
