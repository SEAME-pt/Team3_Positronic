<a id="top"></a>
<p align="right">
  <img src="../assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

<div align="center">

# Templates

<!-- nav:auto -->
<p align="left">
  <a href="README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="standup/template_standup.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="meeting/README.md"><b>Forward &#8594;</b></a>
</p>

</div>

Templates for every document produced in
[`06_project_management/`](../06_project_management/README.md) and
[`07_presentations/`](../07_presentations/README.md). Each template holds **only the structure and the fields
to fill** (`[ … ]`); copy it, fill every field, keep the structure. LaTeX templates use the same style as the
presentation decks ([`presentation/presentation_template.tex`](presentation/presentation_template.tex)):
black canvas, orange `#E86A10` / red `#D93829`, Helvetica bold. 

| Template | When | Copy to |
| :--- | :--- | :--- |
| [sprint/sprint_template.md](sprint/sprint_template.md) | **Monday**, before the planning: the sprint plan; the review table is filled on the Friday. | `06_project_management/sprints/NN_sprint.md` |
| [standup/template_standup.md](standup/template_standup.md) | **Every working day**, 09:30 standup (15 min). | `06_project_management/standups/YYYY-MM-DD.md` |
| [meeting/planning_template.md](meeting/planning_template.md) | **Monday** sprint planning minutes (Scrum Master = chair and scribe). | `06_project_management/meetings/MTG-…-planning.md` |
| [meeting/retro_template.md](meeting/retro_template.md) | **Friday** review & retrospective minutes. | `06_project_management/meetings/MTG-…-retro.md` |
| [presentation/presentation_template.tex](presentation/presentation_template.tex) | Sprint planning (Monday) and review & retrospective (Friday) decks. | `07_presentations/sprint_NN/{planning,review}/present.tex` |

Other templates kept outside this folder:

| Template | When | What it contains |
| :--- | :--- | :--- |
| [Architecture Decision Record](../02_architecture/adr/README.md) | **Whenever a significant, hard-to-reverse technical decision** is made (driver protocol, control architecture, E-Stop design, message interfaces…). One ADR per decision. | ADR: status, context, decision, consequences; numbered, immutable — supersede instead of editing. |

Folder notes: [meeting/](meeting/README.md) · [presentation/](presentation/README.md) · [sprint/](sprint/README.md) · [standup/](standup/README.md).

## Asset paths inside templates

Templates live two directories below `docs/`, so images resolve as `../../assets/<subfolder>/<file>`
(for example the cover logo `../../assets/brand/logo.png` and the tool logos
`../../assets/logos/ros_logo.png`).


## Compiling

Decks are compiled with Docker, in the `texlive/texlive` image, so nothing has to be installed on the
PC. Go into the deck folder and run `latexmk` there.

```bash
cd docs/07_presentations/sprint_01/review                 # the deck you want to compile
docker run --rm -i \
  -v "$PWD":/workdir \
  -v "$PWD/../../../assets":/assets:ro \
  -w /workdir \
  texlive/texlive:latest sh -c "apt-get update -qq && apt-get install -y -qq poppler-utils > /dev/null && mkdir -p _img && latexmk -pdf present.tex && latexmk -c -e '\$clean_ext .= \" %R.nav %R.snm %R.vrb\";' present.tex && pdftoppm -png -r 150 present.pdf _img/present-p && chown -R $(id -u):$(id -g) . _img"
```

The compiled `present.pdf` sits next to `present.tex` and it creates the `_img` folder with all the slides in `.png`.

## Slides as images in the README

The `<embed src="./present.pdf">` in a deck's `README.md` does not render everywhere (VS Code's
Markdown preview, for example, shows images but not PDFs). To show the slides as images, put every page of
`present.pdf` already in `_img` folder and list them in the README.

**1. Check the file names.** `pdftoppm` pads the page number to the number of digits of the last page:
a deck with 9 pages or fewer gives `present-p-1.png`, `present-p-2.png`, …; a deck with 10 to 99 pages gives
`present-p-01.png`, `present-p-02.png`, …

```bash
ls _img/
```

**2. Put them in the deck's `README.md`**, under the slide list, one `<img>` per page:

```html
## Slides as images

<p align="center"><img src="_img/present-p-01.png" width="100%" alt="Slide 1 — Cover"></p>
<p align="center"><img src="_img/present-p-02.png" width="100%" alt="Slide 2 — Abstract"></p>
<p align="center"><img src="_img/present-p-03.png" width="100%" alt="Slide 3 — System Index"></p>
<!-- ... one line per page, up to the last one -->
```

Paths are relative to the deck folder, so the same lines work on GitHub and in the VS Code preview.

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
