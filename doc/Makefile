FIGS = task_life.pdf

all: $(FIGS)
clean:
	rm -f $(FIGS)

%.pdf: %.dot
	 dot -Tpdf -o $@ $^
