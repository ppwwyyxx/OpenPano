TARGET=report
TEX=xelatex -shell-escape
BIBTEX=biber
READER=mupdf

all: try

rebuild output/$(TARGET).pdf: *.tex *.bib output
	cd output && rm -f *.tex *.bib && ln -s ../*.tex ../*.bib .
	cd output &&  $(TEX) $(TARGET).tex && $(BIBTEX) $(TARGET)
	cp output/$(TARGET).pdf ../readme.pdf -f
	killall -HUP mupdf || true

try:
	zsh -c "pgrep -a 'xelatex' || { timeout 10 make rebuild && notify-send 'rebuilt' || notify-send 'build failed' }"

output:
	mkdir output -p
	cd output && rm -f res img code && ln -s ../res ../img ../../code .

view: output/$(TARGET).pdf
	$(READER) output/$(TARGET).pdf &

clean:
	rm -rf output

# compile twice to get cross reference right
final : *.tex *.bib output
	cd output && rm -f *.tex *.bib && ln -s ../*.tex ../*.bib .
	cd output && $(TEX) $(TARGET).tex && $(BIBTEX) $(TARGET) && \
		$(TEX) $(TARGET).tex && $(TEX) $(TARGET).tex

count:
	@word_count *.tex

run: view

.PHONY: all view clean rebuild run
