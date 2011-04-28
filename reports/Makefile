RM=rm
CVS=cvs
LATEX=latex
PDF=ps2pdf
PS=dvips -t letter -Pdownload35 -o 

paper:
	$(LATEX) paper.tex && bibtex paper && latex paper.tex && latex paper.tex 

ps: paper 
	$(PS) paper.ps paper.dvi

pdf: paper ps 
	$(PDF) paper.ps paper.pdf

ci:
	$(CVS) ci *.tex references.bib

clean:
	$(RM) *.out *.blg *.bbl *.dvi *.ps *.log *.aux

release:
	$(LATEX) final.tex && latex final.tex && latex final.tex

release-pdf:
	$(PDF) final.dvi final.pdf

release-ps:
	$(PS) final.ps final.dvi
