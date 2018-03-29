OUTPUT_ARCH(mips)

SECTIONS
{	
	. = 0xff200200 ;
	.text : {*.text}

	. = 0xff202000;
	.data : {*.data}
	.bss : {*.bss}
	_gp = . + 0x800; 

}
