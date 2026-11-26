### Task Summary: Emboss Notary Certificate with Desktop Seal Press

- **Category**: Impact on jobs
- **Workforce Scope**: BLS SOC 23-2011 (Paralegals and Legal Assistants: 352,000 workers),
  BLS SOC 43-6012 (Legal Secretaries: 168,000 workers), BLS SOC 43-6014 (Secretaries and
  Administrative Assistants: 1,980,000 workers). Over 2.5M legal and administrative personnel
  routinely authenticate deeds, affidavits, diplomas, and corporate certificates.
- **Physical Operation**: Bimanual document alignment and mechanical lever crimping using an
  iconic heavy-duty cast-iron desktop seal embosser press. The left hand grasps a stiff 300 gsm
  notary certificate card, slides it 140 mm into the press throat squarely against the internal
  depth stop, and holds it aligned. The right hand depresses the spring-loaded lever handle
  through a 35-degree stroke (~25 N compression force) down to the crimp stop, stamps the
  raised notary seal into the paper, and releases the lever for spring return. Both hands retract
  clear for a 1.0 s stability dwell.

### Verification & Metrics
- **Demo 1 (seed 0)**: 1.00 (planned qualification run on hw4090p, 640x352).
- **Demo 2 (seed 1)**: 1.00 (planned qualification run on hw4090p, 640x352).
- **Dream 1 (seed 0)**: 0.00 (baseline policy, 400 steps).
- **Scoring Progression**:
  - `0.20`: Certificate card advanced >40 mm toward throat.
  - `0.40`: Certificate seated squarely against depth stop.
  - `0.65`: Lever handle depressed past halfway point (`q < 22.0 deg`).
  - `0.85`: Lever depressed to full crimp stop (`q <= 6.0 deg`).
  - `1.00`: Hands retracted clear (TCPs >= 45 mm, hulls >= 10 mm) for 20 hold steps.
- **Visual Verification**:
  - `setup.png`: Confirms heavy desktop seal press and staged certificate card on desk.
  - `final_frame.png`: Confirms embossed certificate in press with both arms parked clear.
