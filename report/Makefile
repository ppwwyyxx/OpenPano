TARGET=report
TEX=xelatex -shell-escape
BIBTEX=biber
READER=mupdf

all: rebuild

rebuild output/$(TARGET).pdf: *.tex *.bib output
	cd output && rm -f *.tex *.bib && ln -s ../*.tex ../*.bib .
	cd output && $(TEX) $(TARGET).tex $(BIBTEX) $(TARGET)


output:
	mkdir output -p
	cd output && rm -f res && ln -s ../res .

view: output/$(TARGET).pdf
	$(READER) output/$(TARGET).pdf &
	(inotifywait -mqe CLOSE_WRITE output/report.pdf | while read; do killall -SIGHUP mupdf; done)

clean:
	rm -rf output

run: view

# compile twice to get cross reference right
final : *.tex *.bib output
	cd output && rm -f *.tex *.bib && ln -s ../*.tex ../*.bib .
	cd output && $(TEX) $(TARGET).tex && $(BIBTEX) $(TARGET) && \
		$(TEX) $(TARGET).tex && $(TEX) $(TARGET).tex
	gs -sDEVICE=pdfwrite -dMaxSubsetPct=100 -dPDFSETTINGS=/ebook -dAutoFilterColorImages=false -dColorImageFilter=/FlateEncode -sOutputFile="../report.pdf" -dNOPAUSE -dBATCH output/$(TARGET).pdf

count:
	@word_count *.tex

dist: output/$(TARGET).pdf
	rm -rf paper
	mkdir paper
	cp output/$(TARGET).pdf paper/
	cp -r screenshots paper/
	7z a -tzip paper.zip paper
	rm -rf paper

.PHONY: all view clean rebuild dist
