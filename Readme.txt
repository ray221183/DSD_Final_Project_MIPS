DSD_Final_Project_MIPS_G8/
	src/
		Baseline/
			rtl/
				CHIP.v                Baseline優化後的CHIP.v
			syn/
				CHIP_syn.v            Cycle time=2.9ns
				CHIP_syn.sdf          Cycle time=2.9ns
				CHIP_syn.ddc          Cycle time=2.9ns
		Extension/
			BrPred/
				rtl/
					CHIP.v            BrPred的CHIP.v，採用2-level 2-bit branch prediction with local history table
				syn/
					CHIP_syn.v        Cycle time=2.8ns
					CHIP_syn.sdf      Cycle time=2.8ns
					CHIP_syn.ddc      Cycle time=2.8ns
			MultDiv/
				rtl/
					CHIP.v            MultDiv的CHIP.v，採用iterative approach
				syn/
					CHIP_syn.v        Cycle time=2.8ns
					CHIP_syn.sdf      Cycle time=2.8ns
					CHIP_syn.ddc      Cycle time=2.8ns
			L2Cache/
				rtl/
					CHIP_L2Cache.v    L2Cache的CHIP.v，採用separate、dm、write back 
				syn/
					CHIP_syn.v        Cycle time=4.7ns
					CHIP_syn.sdf      Cycle time=4.7ns
					CHIP_syn.ddc      Cycle time=4.7ns   
	Report.pdf
	Presentation.pptx
	DSD_Final_Project_Scores_MIPS.pdf
	Readme.txt

