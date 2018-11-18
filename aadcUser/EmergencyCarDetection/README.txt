For proper work of the filter EmergencyCarDetection make sure:

1. A file with name "rawCompareSignal_siren.txt" exist
2. the rawCompareSignal_siren.txt file include 2048 lines of float values (values should be normal, e.g. not begin with a lot of zeros)
3. #define BOOL_RECORD_COMPARE_SIGNAL is commented (auskommentiert!) in EmergencyCarDetection.h

If that is not the case:
1. make sure #define BOOL_RECORD_COMPARE_SIGNAL is commented (auskommentiert!) in EmergencyCarDetection.h
2. delete the file "rawCompareSignal_siren.txt"
3. copy the file "rawCompareSignal_siren_SAVE(High).txt" and rename it to "rawCompareSignal_siren.txt"