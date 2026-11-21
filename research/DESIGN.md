# Canvas rewrite: design plan and review

The reader is discovering what autonomous-flight problems exist and what independently identifiable projects solve them. Their broad idea is intentional, not an incomplete requirements checklist. Do not score project coverage against an imagined specification.

## Model before layout

One underlying record per independently identifiable project. Two ways to encounter it: an assembled system with its documented project dependencies exposed, or a focused contribution whose relationships unfold on request. Hybrid projects may appear in both contexts. Repeated appearances are portals to the same record, not duplicated research. Internal directories are described within the parent; they do not become independent project nodes. Reference-only and unreleased entries remain searchable but are not presented as released assemblies.

Separate integration, optional integration, copied/adapted code, intellectual ancestry and evaluation. A line must say which relationship was established. Never turn every README acknowledgement into a runtime dependency. A graph is the documented portion of an assembly, not a complete dependency lockfile.

## Visual plan

Palette: cloud #eef4f8, white #ffffff, ink #16354b, route blue #245bd7, sensing teal #187b79, caution ochre #a75c25. Retain Barlow for project names and Source Sans 3 for explanations, embedded for offline use.

The canvas fills the viewport. Compact floating controls sit around it: two-mode switch and search at the top, problem index on the left, selected-project inspector on the right, pan/zoom/fit controls below. A small reference menu contains audit, methodology and download actions. Comparison is a contextual tray/sheet, not primary navigation.

Assembled view: several spatially separated graph clusters, each introduced by the actual problem the system tackles. Root nodes carry readable purpose and release status. Constituent nodes are independently linkable projects. Zooming into a cluster reveals its labels and secondary relationships; selecting any appearance opens the same project record. Shared-project links are revealed on demand, avoiding a global hairball.

Focused view: a landscape of contribution nodes grouped by plain-language problems (finding position, representing obstacles, choosing motion, keeping flight under control, learning behaviors, testing systems). Selecting a node explains its contribution. Expanding reveals real dependencies and consumers; jumping to an assembled system preserves a clear route back.

[mode switch                  search        reference menu]
[problem index       pannable project canvas       inspector]
[                  clusters / contributions                 ]
[context / history                    zoom, fit     comparison]

Mobile: canvas remains primary with a compact mode control; index and details become deliberate sheets. Pinch/pan and zoom buttons both work. Keyboard offers search/list access, node activation and canvas panning, with visible focus and a reduced-motion path.

## Review against brief

Reject the old six-page tool organization and a renamed version of the same center-and-spoke diagram. Use real multi-project clusters with drill-down, a persistent world transform and local expansion. Reject automatic performance rankings, inferred capability percentages and a mandatory setup wizard. Keep the historical audit secondary. Preserve uncertainty, sources and licensing in context. Do not hide a focused contribution because it occurs within an assembly.

The bold visual decision is the spatial project landscape. Keep other surfaces quiet; no oversized hero, ornamental flight instruments or animated force physics. The canvas should teach through clear problem statements and controlled disclosure, rather than asking the reader to decipher unexplained dots.

## Verification

Check data identity and edge evidence; no internal-only directory nodes; no removed-topic text in the deliverable; source links for new relationships. Test pointer pan, cursor-anchored wheel zoom, fit, focus, reset, expansion/collapse, two-mode navigation, search, comparison, audit, exports, keyboard and mobile. Inspect desktop and mobile screenshots and verify offline operation.
