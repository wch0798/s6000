from pathlib import Path
import re, sys
p=Path(__file__).with_name('BALLY_S6000_UNO_R4_MINIMA_V6_5_14_RAM_AUTO_CLEAR.ino')
s=p.read_text()
checks=[]
def ck(name, cond): checks.append((name,bool(cond)))
ck('build id v6.5.14', 'V6.5.14' in s)
ck('runtime clear helper exists', 'void v6ClearRuntimeTransaction(const char *reason)' in s)
helper=s[s.index('void v6ClearRuntimeTransaction'):s.index('V6MeterSnapshotStored v6StoreMeters')]
ck('clear invalidates journal', 'v6LatestJournalValid = false;' in helper)
ck('clear drops recovery', 'v6RecoveryRequired = false;' in helper)
ck('clear does not touch EEPROM', 'EEPROM.' not in helper)
ck('stable no-move proof helper', 'bool v6ConfirmStableCreditExact(uint64_t target, uint8_t samples)' in s)
stable=s[s.index('bool v6ConfirmStableCreditExact'):s.index('bool v6SyncAftCycleBeforeValueCommand')]
ck('stable proof uses LP1A', 'readCurrentCredits1A(now, false)' in stable)
ck('stable proof 3 calls used', s.count('v6ConfirmStableCreditExact(') >= 3)
# deposit immediate failure branch
start=s.index('bool executeCardDepositTransaction()')
end=s.index('// Explicit declaration required before the new V6.4 AFT helpers.', start)
dep=s[start:end]
ck('deposit exact meter authority', 'waitForExactCreditMeter(expectedCredits' in dep)
ck('deposit no-move tests PRE', 'afterCredits == beforeCredits' in dep)
ck('deposit no-move stable proof', 'v6ConfirmStableCreditExact(beforeCredits, 3)' in dep)
ck('deposit no-move clears RAM', 'v6ClearRuntimeTransaction("DEPOSIT NO MOVEMENT")' in dep)
ck('deposit no-move clears input', 'inputValue = 0;' in dep)
ck('deposit ambiguous retains recovery', 'DEPOSIT AMBIGUOUS' in dep and 'RECOVERY_REQUIRED' in dep)
ck('deposit failure never retries 8A', dep.count('sendLegacyBonus8A(') == 1)
# recovery RFID_ATOMIC branch
r0=s.index('} else if (type == V6JournalType::RFID_ATOMIC) {', s.index('void v6ServiceRecovery()'))
r1=s.index('} else if (type == V6JournalType::HANDPAY)', r0)
rr=s[r0:r1]
ck('rfid recovery expected success path', 'current == expected' in rr)
ck('rfid recovery no-move PRE path', 'current == before.currentCredits' in rr)
ck('rfid recovery stable no-move proof', 'v6ConfirmStableCreditExact(before.currentCredits, 3)' in rr)
ck('rfid recovery auto-clears RAM', 'v6ClearRuntimeTransaction("RFID DEPOSIT NO MOVEMENT")' in rr)
ck('rfid recovery no 8A resend', 'sendLegacyBonus8A' not in rr)
ck('rfid recovery ambiguous no manual-review loop text', 'meter not exact; no resend, manual review required' not in rr)
# cashout rejection clearing
ck('AFT immediate reject clears RAM', 'v6ClearRuntimeTransaction("AFT REJECTED / NO MOVEMENT")' in s)
ck('cashout recovery reject clears RAM', 'v6ClearRuntimeTransaction("RFID CASHOUT REJECT / NO MOVEMENT")' in s)
ck('cashout FF reject clears RAM', 'v6ClearRuntimeTransaction("RFID CASHOUT FF REJECT / NO MOVEMENT")' in s)
ck('split reject clears RAM', 'v6ClearRuntimeTransaction("SPLIT REJECTED / NO MOVEMENT")' in s)
# EEPROM policy
write=s[s.index('bool v6WriteJournal'):s.index('V6MeterSnapshotStored v6StoreMeters')]
ck('transaction write function RAM only', 'EEPROM.' not in write)
ck('boot does not load legacy tx EEPROM', 'void v6LoadJournal()' in s and 'EEPROM.get' not in s[s.index('void v6LoadJournal()'):s.index('bool v6WriteJournal')])
ck('profile EEPROM puts only two', s.count('EEPROM.put(') == 2)
# 12-bit calibration preserved
ck('normal setup is 12 bit', 'analogReadResolution(12);' in s)
for key,val in [('#',931),('1',1081),('9',1598),('*',1738),('2',1817),('0',2000),('8',2224),('7',2356),('4',2677),('3',3109),('6',3385),('5',4095)]:
    ck(f'key center {key}={val}', str(val) in s)
# model of new RFID deposit recovery decision
def decide(pre, expected, now, stable):
    if now == expected: return 'SUCCESS'
    if now == pre and stable: return 'FAILED_CLEAR'
    return 'AMBIGUOUS_RECOVERY'
ck('model 0->1 stays 0 auto-clear', decide(0,1,0,True)=='FAILED_CLEAR')
ck('model 250->251 stays 250 auto-clear', decide(250,251,250,True)=='FAILED_CLEAR')
ck('model reaches expected success', decide(0,1,1,True)=='SUCCESS')
ck('model intermediate/other ambiguous', decide(10,15,12,True)=='AMBIGUOUS_RECOVERY')
ck('model unstable PRE remains recovery', decide(0,1,0,False)=='AMBIGUOUS_RECOVERY')
passed=sum(x for _,x in checks)
for name,ok in checks:
    print(('PASS' if ok else 'FAIL')+': '+name)
print(f'VALIDATION {passed}/{len(checks)}')
if passed != len(checks): sys.exit(1)
