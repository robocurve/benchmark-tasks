# Novelty: the ability this task evaluates

This submission evaluates **closure-preserving extraction of a preloaded flexible liner from its rigid bin, with continuous content retention and a verified released end state**. The job outcome is a closed load removed from the bin and resting beside it. The comparison below states the distinction at the level of initial conditions, manipulated constraints and measurable outcomes.

| Aspect | This task | Closest confirmed prior art: TRI `PackItemsIntoStringBag` |
|---|---|---|
| Starting state | Open liner already loaded inside a surrounding rigid bin. | Bag and items prepared for a packing task. |
| Main object interaction | Opposed drawcord tension gathers the film; the loaded liner must leave the surrounding bin with its contents retained. | Opens a bag, prepares and packs a bottle, ball and sunglasses, then grasps/pulls two drawstrings to close it. |
| Closure criterion | Mouth must reach ≤80 mm (smaller than the 83 mm diagonal of any block's smallest face), be ≤80 mm when the whole bag first clears the rim by 20 mm and over the final 2 s, with no lock or knot; all contents must stay inside at every step. | Published milestones end in drawstring closure of the packed bag. |
| Removal criterion | Every bag/content point first clears the rim by ≥20 mm; in the final state the bag rests ≥20 mm clear of the pot and the table edges. | The listed milestones do not specify extraction from a surrounding bin. |
| Finished work | Set down upright on the table (mouth ≥150 mm above it), ≥20 mm clear of the pot and table edges, opening ≤80 mm, grippers ≥60 mm clear, bag bounds moving ≤10 mm over the final 2 s. | The listed milestones do not specify that post-extraction release/rest check. |

The proposed distinction is maintaining a gathered flexible enclosure and its load through the change from bin support to robot support to table support. The score awards 0.25 for cinching, 0.5 for carrying the closed load clear of the bin, 0.75 for setting it down upright beside the bin and 1.0 for a stable released result. Content escape or final reopening defeats success.

**Overlap is acknowledged:** TRI `PackItemsIntoStringBag` already closes a loaded flexible bag with two drawstrings; this task adds extracting the closed, loaded liner from a taped bin and setting it down without losing contents.

## Other relevant comparisons

| Prior task | Published overlap | Difference in the stated goal |
|---|---|---|
| BEHAVIOR-1K `emptying_trash_cans` | Loose waste transfer from bins into sacks. | Does not require cinching and removing the already loaded liner from its bin. |
| BEHAVIOR-1K `taking_trash_outside` | Moving existing trash bags to the garden. | Does not require an open-to-cinched mouth transition. |
| Gemini Robotics 2 “Tie trash bag” | Directly relevant bag-closing evaluation. | Public setup/closure details are insufficient to exclude equivalence; no assumption that it uses a knot. |
| Global Innovation Challenge 2025, Task 6 Cleaning | Taking out/tying a bin bag, replacement and carrying a 4 kg bag outside, using human-piloted wearable assistance. | Different evaluation and embodiment; this is workflow prior art, and embodiment alone is not a novelty claim. |
| RoboCurve PR #385 `share-one-trash-can` | Arms deposit separate loose items through a shared bin opening. | Concerns shared-opening coordination, rather than closing and extracting a liner. |

Public keyword searches are incomplete; no claim of exclusive ownership or “no close matches” is made. The organiser's comparison packet and judgment remain authoritative.

## Sources checked 15 November 2026

- TRI, *A Systematic Study of Data Modalities and Strategies for Co-training Large Behavior Models for Robot Manipulation*, Appendix 7B and Fig. S18: https://arxiv.org/html/2602.01067v1#A7.SS2 ; real-robot project: https://co-training-lbm.github.io/
- https://behavior.stanford.edu/knowledgebase/tasks/emptying_trash_cans-0.html
- https://behavior.stanford.edu/knowledgebase/tasks/taking_trash_outside-0.html
- https://deepmind.google/models/gemini-robotics/vla/
- https://global-innovation-challenge.com/document/2025/GIC2025_Application%20Guidelines_EN_20250708.pdf
- https://github.com/robocurve/benchmark-tasks/pull/385
