# writing and code rules

## Vocabulary blacklist

Do not use these words or their variants. If a sentence cannot be written without one, restructure the sentence.

- delve, tapestry, intricate, testament, pivotal, crucial, robust, vibrant,
  multifaceted, underscore, highlight, showcase, foster, bolster, align,
  nuanced, landscape (as metaphor), meticulous, invaluable, garner, profound,
  seamless, enduring.

## Structural bans

- No trailing participles. Do not end sentences with summary clauses ("...highlighting the importance of X").
- No negative parallelisms ("not just X but also Y").
- No rule of three by reflex. List as many items as the content needs, no more.
- No copula avoidance. Use `is`, `are`, `was`, `were`. Do not substitute with `serves as`, `stands as`, `acts as`, `represents`, `embodies`.
- No formulaic transitions. Do not open paragraphs with `Additionally`, `Consequently`, `In summary`, `Overall`.
- No puffery. Do not write "stands as a testament", "marks a pivotal moment", "revolutionizes", "paves the way".
- No outline conclusions. Do not open closing paragraphs with "In conclusion" or "Looking ahead."

## Tone

- State facts. Do not announce significance. Provide the number; let the reader conclude.
- No vague attributions. Do not write "researchers note", "experts argue", "it has been shown that". Cite the specific source.
- No collaborative filler: "we acknowledge that", "we recognize that".
- No apologetic framing: "preliminary results", "remains an open question", "future work will".
- No elegant variation. Repeat the technical term. Do not invent synonyms to avoid repetition.

## Formatting

- Sentence-case section headings. No Title Case.
- No bold for emphasis inside paragraph prose. Reserve bold for table column heads and contribution titles in itemized lists.
- No bolded inline bullet points (`- **Term:** Definition`). Use a proper paragraph or properly formatted list.
- No em dashes. Use commas, parentheses, or colons instead. Use -- only when no punctuation rewrite works.
- Straight quotes only. No curly/smart quotes, no non-breaking spaces.
- Only characters typeable on a standard keyboard. No Unicode punctuation substitutes.
- Do not write inline-header vertical lists. Use real section or paragraph headings.

## Slides and deliverables

- Do not build bespoke HTML or JS slide decks, or web artifacts, for slides, documents, or reports. For slides, write Marp markdown in the `slide/` folder (one `.md` per deck) that renders with the Marp CLI. Use a built-in Marp theme, not hand-written CSS.
- Avoid the signs of AI-generated design. Do not use: emoji as section markers or decoration, gradient hero banners, everything centered, purple-to-blue or acid-accent color schemes, oversized display type, decorative numbered markers (01 / 02 / 03) that do not encode a real sequence, or "revolutionizing / pioneering / seamless" copy. Plain, factual, and typographically simple is the target.

---

## Code

- Prioritize correctness and clarity. Speed and efficiency are secondary unless specified.
- Comments explain why, not what. Well-named identifiers describe what the code does. Only add a comment when there is a non-obvious constraint, a workaround for a specific bug, or a subtle invariant a reader would otherwise miss.
- Prefer adding functionality to existing files. Only create a new file when it is a genuinely new logical component.
- Avoid creative additions unless explicitly requested.
- Use full words for names. No single-letter variables (except conventional loop indices like `i`, `j`) and no abbreviations like `q` for queue, `r` for response, `e` for error.
- Keep only the current approach. When a decision replaces an earlier one, delete the old code, options, and fallbacks; do not leave superseded paths, alternatives, or dead branches behind.
- No temporary or transition comments. Do not write comments that narrate history ("this used to be X", "now uses Y instead of Z") or the process of change. A comment states a why that is true of the code as it stands.
- Name things for what they are, not their novelty or version. No `new`, `clean`, `v2`, `final`, `spatial2`, or `_old` in file, symbol, or output names. When an approach is replaced, rename in place so the canonical name always points at the current thing.

## Error handling

- Never let errors fail silently. Don't catch exceptions and swallow them without logging or re-raising.
- Never discard return values from fallible operations without handling the failure case.
- Validate inputs at system boundaries (user input, external APIs, file I/O). Trust internal code and framework guarantees.
- When async operations fail, errors must propagate so users receive meaningful feedback.

## Python

- Annotate all function signatures with type hints.
- Propagate errors with `raise`. Don't return `None` to signal failure: that is silent error swallowing.
- Use `pathlib.Path` for file path operations instead of string concatenation.
- Don't use bare `except: pass` or `except Exception: pass` without at minimum logging the exception.

## Git

- No co-author trailers in commit messages. Do not add `Co-Authored-By: Claude` or any other generated-with attribution lines.

## Module structure

- Python: don't create `__init__.py` barrel files that only re-export from submodules.
- Python: prefer flat module files (`src/some_module.py`) over package directories (`src/some_module/__init__.py`) unless the module genuinely warrants its own directory.