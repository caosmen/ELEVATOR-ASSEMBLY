/* |------------------------------------------------------------------------| */
/* | Implementação Elevador (Firmware)                                      | */
/* |------------------------------------------------------------------------| */
/* | Curso: Engenharia de Computação										| */
/* | Disciplina: Microcontroladores e Aplicações                            | */
/* | Professor: Erick Barboza                                               | */
/* | Alunos: Bruno Lemos, Karla Sophia, Leticia Gabriela e Maria Fernanda   | */
/* |------------------------------------------------------------------------| */

/* ---------------------Definição das variáveis globais---------------------- */
#define CLOCK 16.0e6 
#define DELAY_TIMER_A 1.0

#define PRESCALE_TIMER_A 0b101
#define PRESCALE_DIV_TIMER_A 1024
#define WGM_TIMER_A 0b0100
/* ---------------------Definição das variáveis globais---------------------- */

/* -------------------------------------------------------------------------- */

/* ------------------------Definição dos registradores----------------------- */
// Registrador para uso geral
.def global_temp = r16

// Registradores usados para armazenar as informações do elevador
.def global_state = r20
.def global_current_floor = r21
.def global_next_floor = r22
.def global_direction = r23
.def global_counter = r24
/* ------------------------Definição dos registradores----------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------Definição dos pinos usados----------------------- */
.equ pin_button_external_floor_0 = PB5
.equ pin_button_external_floor_1 = PB4
.equ pin_button_external_floor_2 = PB3
.equ pin_button_external_floor_3 = PB2

.equ pin_button_internal_floor_0 = PB1
.equ pin_button_internal_floor_1 = PB0
.equ pin_button_internal_floor_2 = PD7
.equ pin_button_internal_floor_3 = PD6

.equ pin_button_open_door = PD5
.equ pin_button_close_door = PD4

.equ pin_buzzer = PC5
.equ pin_led = PC4
/* -------------------------Definição dos pinos usados----------------------- */

/* -------------------------------------------------------------------------- */

/* -----------------------Definição da fila de chamadas---------------------- */
.dseg
queue: .BYTE 4
/* -----------------------Definição da fila de chamadas---------------------- */

/* -------------------------------------------------------------------------- */

/* -----------------------Alteração da memória para cseg--------------------- */
.cseg
.org 0
/* -----------------------Alteração da memória para cseg--------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------Tabela de interrupção (IVT)---------------------- */
rjmp SETUP
.org OC1Aaddr
jmp OCI1A_Interrupt
/* -------------------------Tabela de interrupção (IVT)---------------------- */

/* -------------------------------------------------------------------------- */

/* --------------------------------Interrupções------------------------------ */
OCI1A_Interrupt:
	push global_temp
	in global_temp, SREG
	push global_temp

	// Incrementando o contador
	inc global_counter

	pop global_temp
	out SREG, global_temp
	pop global_temp

	reti
/* --------------------------------Interrupções------------------------------ */

/* -------------------------------------------------------------------------- */


/* ----------------------------------FUNÇÕES--------------------------------- */


/* ---------Clear counter: Zera o contador e o registrador de contagem------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (global_temp)                                    */
/* Definição: void Clear_counter()                                            */
Clear_counter:
	push global_temp

	// Zerando o contador
	clr global_counter

	// Zerando o registrador de contagem
	clr global_temp
	sts TCNT1L, global_temp
	sts TCNT1H, global_temp

	pop global_temp

	ret
/* -------------------------------------------------------------------------- */

/* ----------------Add Call: Adiciona uma nova chamada na fila--------------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (global_temp), r17, r18                          */
/* Definição: void Add_call(floor_called int, external_or_internal_floor int) */
/* Argumentos: floor_called (r17), external_or_internal_floor (r18)			  */
.def floor_called = r17
.def external_or_internal_floor = r18
Add_call:
	push global_temp

	// Inicialização do ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Desloca o ponteiro para a posição do andar na fila
	clr global_temp

	add ZL, floor_called
	adc ZH, global_temp

	// Lê o dado armazenado na posição da fila e salva no respectivo registrador
	ld global_temp, Z

	// Salva no valor lido da chamada atual (interna: 0b10 ou externa: 0b01)
	// O operador "or" é usado para mesclar e armazenar as chamadas internas e externas
	or global_temp, external_or_internal_floor

	// Salva o novo valor na posição da fila
	st Z, global_temp

	pop global_temp

	ret

.undef floor_called
.undef external_or_internal_floor
/* -------------------------------------------------------------------------- */

/* -----------Clear Call: Limpa a chamada que foi executada da fila---------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (global_temp)                                    */
/* Definição da função: void Clear_call()                                     */
Clear_call:
	push global_temp

	// Inicialização do ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Desloca o ponteiro para a posição do andar na fila
	clr global_temp

	add ZL, global_current_floor
	adc ZH, global_temp

	// Salva o novo valor na posição da fila (limpa a posição da fila)
	clr global_temp
	st Z, global_temp

	pop global_temp

	ret
/* -------------------------------------------------------------------------- */

/* Get high priority floor: Atribui ao registrador next_floor o novo destino  */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (global_temp), r17, r18, r19                     */
/* Registradores usados: r22 (global_current_floor), r24 (global_direction)   */
/* Definição da função: int Get_next_floor()                                  */
/* Retorno: r19 (andar com maior prioridade)                                  */
.def floor_queue_position = r17
.def call_in_floor = r18
.def next_destiny = r19
Get_high_priority_floor:
	push global_temp
	push floor_queue_position
	push call_in_floor

	// Inicialização do ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Switch case para os casos em que o elevador está parado, subindo ou descendo
	switch_direction:
		// Caso o elevador esteja parado
		cpi global_direction, 0
		breq case_direction_stop
		// Caso o elevador esteja subindo
		cpi global_direction, 1
		breq case_direction_up
		// Caso o elevador esteja descendo
		cpi global_direction, -1
		breq case_direction_down

	case_direction_stop:
		// Desloca o ponteiro para a terceira posição
		adiw Z, 3

		// Inicializa a posição da fila no andar de maior prioridade (3º andar) (loop de 3 a 0)
		ldi floor_queue_position, 3
		while_direction_stop:
			// Verifica se já passou por todos os valores
			cpi floor_queue_position, 0
			brlt end_while_direction_stop

			// Lê a posição atual da fila
			ld call_in_floor, Z

			// Decrementa o ponteiro Z
			subi ZL, 1
			sbci ZH, 0

			// Verifica se existe uma chamada
			cpi call_in_floor, 0
			// Caso não exista, ele pula o retorno e checa o próximo andar
			breq skip_direction_stop

			rjmp found_result

			skip_direction_stop:
				dec floor_queue_position
				rjmp while_direction_stop
		end_while_direction_stop:

		rjmp end_switch_direction

	case_direction_up:
		clr global_temp
		// Desloca o ponteiro para a posição do andar atual
		add ZL, global_current_floor
		adc ZH, global_temp

		// Inicializa a posição da fila no andar atual
		mov floor_queue_position, global_current_floor
		while_direction_up_internal:
			// Verifica se já passou por todos os valores da fila (andares)
			cpi floor_queue_position, 4
			brge end_while_direction_up_internal

			// Lê a posição atual da fila
			ld call_in_floor, Z+

			// Faz uma máscara no bit referente à chamada interna
			andi call_in_floor, 0b00000010
			// Verifica se existe uma chamada interna
			cpi call_in_floor, 0
			// Caso não exista, ele pula o retorno e checa o próximo andar
			breq skip_direction_up_internal

			rjmp found_result

			skip_direction_up_internal:
				inc floor_queue_position
				rjmp while_direction_up_internal
		end_while_direction_up_internal:

		// Reseta a posição do ponteiro Z
		ldi ZL, low(queue)
		ldi ZH, high(queue)

		// Desloca o ponteiro para a terceira posição
		adiw Z, 3

		// Inicializa a posição da fila no andar de maior prioridade (3º andar) (loop de 3 ao andar atual)
		ldi floor_queue_position, 3
		while_direction_up_external:
			// Verifica se já passou por todos os valores (andares)
			cp floor_queue_position, global_current_floor
			brlt end_while_direction_up_external

			// Lê a posição atual da fila
			ld call_in_floor, Z

			// Decrementa o ponteiro Z
			subi ZL, 1
			sbci ZH, 0

			// Faz uma máscara no bit referente à chamada externa
			andi call_in_floor, 0b00000001
			// Verifica se existe uma chamada externa
			cpi call_in_floor, 0
			// Caso não exista, ele pula o retorno e checa o próximo andar
			breq skip_direction_up_external

			rjmp found_result

			skip_direction_up_external:
				dec floor_queue_position
				rjmp while_direction_up_external
		end_while_direction_up_external:

		rjmp end_switch_direction

	case_direction_down:
		clr global_temp
		// Desloca o ponteiro para a posição do andar atual
		add ZL, global_current_floor
		adc ZH, global_temp

		// Inicializa a posição da fila no andar atual
		mov floor_queue_position, global_current_floor
		while_direction_down:
			// Verifica se já passou por todos os valores (andares)
			cpi floor_queue_position, 0
			brlt end_while_direction_down

			// Lê a posição atual da fila
			ld call_in_floor, Z

			// Decrementa o ponteiro Z
			subi ZL, 1
			sbci ZH, 0

			// Verifica se existe uma chamada
			cpi call_in_floor, 0
			// Caso não exista, ele pula o retorno e checa o próximo andar
			breq skip_direction_down

			rjmp found_result

			skip_direction_down:
				dec floor_queue_position
				rjmp while_direction_down
		end_while_direction_down:

		rjmp end_switch_direction
	end_switch_direction:

	// Não encontrou nenhum andar para ir
	ldi next_destiny, -1
	rjmp not_found_result

	found_result:
		// Atribui o andar encontrado ao destino
		mov next_destiny, floor_queue_position

	not_found_result:
		
	pop call_in_floor
	pop floor_queue_position
	pop global_temp

	ret

.undef floor_queue_position
.undef call_in_floor
.undef next_destiny
/* -------------------------------------------------------------------------- */

/* -------Update display: Uma subrotina que atualiza o andar no display------ */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (global_temp), r22 (global_current_floor)        */
Update_display:
	push global_temp

	// Copia o andar atual para o registrador temporário
	mov global_temp, global_current_floor
	// Move o andar atual para a posição do pino 2 e 3 da Porta D
	lsl global_temp
	lsl global_temp

	// Escreve o novo valor do display na Porta D
	out PORTD, global_temp

	pop global_temp

	ret
/* -------------------------------------------------------------------------- */

/* -----------Delay 20ms: Uma subrotina que aciona um delay de 20ms---------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (global_temp), r17, r18                          */
/* Assumindo que o clock é 16 Mhz, para 20 ms são necessários 320.000 ciclos  */
/* CLOCK (16.0e6) * 20ms = CLOCK (16.0e6) * 0.02 = 320.000                    */
.def temp_2 = r17
.def temp_3 = r18
Delay_20ms:
	push global_temp
	push temp_2
	push temp_3

	// Calcula a quantidade de ciclos que o loop abaixo deve ser executado
	// Como no loop cada iteração vai consumir cinco ciclos, precisamos dividir os 320.000 ciclos por cinco
	.equ required_cycles = int(CLOCK * 0.02 / 5.0)

	// Salva os valores em três registradores
	ldi temp_3, byte3(required_cycles)
	ldi temp_2, high(required_cycles)
	ldi global_temp, low(required_cycles)

	// Executa os 320.000 cliclos
	subi global_temp, 1
	sbci temp_2, 0
	sbci temp_3, 0
	brcc pc-3

	pop temp_3
	pop temp_2
	pop global_temp

	ret

.undef temp_2
.undef temp_3
/* -------------------------------------------------------------------------- */

/* ----------------------------------FUNÇÕES--------------------------------- */

/* -----------------------------CONFIGURAÇÃO INICIAL ------------------------ */
SETUP:
	/* ------------------------Inicialização da pilha------------------------ */
	ldi global_temp, high(RAMEND)
	out SPH, global_temp
	ldi global_temp, low(RAMEND)
	out SPL, global_temp
	/* ------------------------Inicialização da pilha------------------------ */

	/* ---------------------------------------------------------------------- */

	/* ---------------------Definição das portas B, D e C-------------------- */

	/* Porta B                                                                */
	/*                                                                        */
	/* PB5 = Andar 0 (Botões externos) - Entrada                              */
	/* PB4 = Andar 1 (Botões externos) - Entrada                              */
	/* PB3 = Andar 2 (Botões externos) - Entrada                              */
	/* PB2 = Andar 3 (Botões externos) - Entrada                              */
	/* PB1 = Andar 0 (Botões internos) - Entrada                              */
	/* PB0 = Andar 1 (Botões internos) - Entrada                              */
	ldi global_temp, 0b00000000

	// Configura a Porta B
	out DDRB, global_temp

	/* Porta D                                                                */
	/*                                                                        */
	/* PD7 = Andar 2 (Botões internos) - Entrada                              */
	/* PD6 = Andar 3 (Botões internos) - Entrada                              */
	/* PD5 = Abrir Porta			   - Entrada                              */
	/* PD4 = Fechar Porta 			   - Entrada                              */
	/* PD3 = Display 7 seg (Entrada B) - Saída     		                      */
	/* PD2 = Display 7 seg (Entrada A) - Saída         		                  */
	ldi global_temp, 0b00001100

	// Configura a Porta D
	out DDRD, global_temp

	/* Porta C                                                                */
	/*                                                                        */
	/* PC5 = Buzzer 				   - Saída                                */
	/* PC4 = Led 					   - Saída                                */
	ldi global_temp, 0b00110000

	// Configura a Porta C
	out DDRC, global_temp
	/* ---------------------Definição das portas B, D e C-------------------- */

	/* ---------------------------------------------------------------------- */

	/* ------------Definição do estado inicial dos pinos de Saída------------ */

	/* Porta C                                                                */
	/*                                                                        */
	/* PC5 = Buzzer 					- 0                                   */
	/* PC4 = Led 						- 0                                   */
	cbi PORTC, pin_buzzer
	cbi PORTC, pin_led
	/* ------------Definição do estado inicial dos pinos de Saída------------ */

	/* ---------------------------------------------------------------------- */

	/* -------------Definição do estado inicial dos registradores------------ */

	// Atribui os valores aos registradores
	ldi global_state, 1
	ldi global_current_floor, 0
	ldi global_next_floor, -1
	ldi global_direction, 0
	ldi global_counter, 0

	// Atualiza o display para o andar inicial
	call Update_display
	/* -------------Definição do estado inicial dos registradores------------ */

	/* ---------------------------------------------------------------------- */

	/* -------------------------Inicialização da fila------------------------ */
	clr global_temp

	sts queue, global_temp
	sts queue + 1, global_temp
	sts queue + 2, global_temp
	sts queue + 3, global_temp
	/* -------------------------Inicialização da fila------------------------ */

	/* ---------------------------------------------------------------------- */

	/* ------------------------Configuração do Timer A----------------------- */
	
	// Calcula o valor para o TOP
	.equ TOP = int(0.5 + ((CLOCK / PRESCALE_DIV_TIMER_A) * DELAY_TIMER_A))

	// Valida se ele está dentro do limite esperado (65.535)
	.if TOP > 65535
	.error "TOP is out of range"
	.endif

	// Atribui o valor de comparação (TOP)
	ldi global_temp, high(TOP)
	sts OCR1AH, global_temp
	ldi global_temp, low(TOP)
	sts OCR1AL, global_temp

	// Atribui o valor para o modo de operação (WGM)
	ldi global_temp, ((WGM_TIMER_A & 0b11) << WGM10)
	sts TCCR1A, global_temp
	ldi global_temp, ((WGM_TIMER_A >> 2) << WGM12) | (PRESCALE_TIMER_A << CS10)
	sts TCCR1B, global_temp

	// Habilita o bit na posição OCIE1A
	lds global_temp, TIMSK1
	sbr global_temp, 1 << OCIE1A
	sts TIMSK1, global_temp

	sei
	/* ------------------------Configuração do Timer A----------------------- */

	rjmp MAIN
/* -----------------------------CONFIGURAÇÃO INICIAL ------------------------ */

/* -------------------------------------------------------------------------- */

/* ---------------------------FUNCIONAMENTO PRINCIPAL------------------------ */
MAIN:
	.def floor_called = r17
	.def external_or_internal_floor = r18
	.def next_destiny = r19

	/* ----------------Leitura dos botões (internos e externos)-------------- */
	sbic PINB, pin_button_external_floor_0
	rjmp button_external_floor_0_pressed

	sbic PINB, pin_button_external_floor_1
	rjmp button_external_floor_1_pressed

	sbic PINB, pin_button_external_floor_2
	rjmp button_external_floor_2_pressed

	sbic PINB, pin_button_external_floor_3
	rjmp button_external_floor_3_pressed

	sbic PINB, pin_button_internal_floor_0
	rjmp button_internal_floor_0_pressed

	sbic PINB, pin_button_internal_floor_1
	rjmp button_internal_floor_1_pressed

	sbic PIND, pin_button_internal_floor_2
	rjmp button_internal_floor_2_pressed

	sbic PIND, pin_button_internal_floor_3
	rjmp button_internal_floor_3_pressed
	/* ----------------Leitura dos botões (internos e externos)-------------- */

	rjmp switch_state

	/* ------Preparação dos atributos para executar a função (Add_call)------ */
	button_external_floor_0_pressed:
		ldi floor_called, 0
		ldi external_or_internal_floor, 1
		
		rjmp button_pressed

	button_external_floor_1_pressed:
		ldi floor_called, 1
		ldi external_or_internal_floor, 1
		
		rjmp button_pressed

	button_external_floor_2_pressed:
		ldi floor_called, 2
		ldi external_or_internal_floor, 1
		
		rjmp button_pressed

	button_external_floor_3_pressed:
		ldi floor_called, 3
		ldi external_or_internal_floor, 1
		
		rjmp button_pressed

	button_internal_floor_0_pressed:
		ldi floor_called, 0
		ldi external_or_internal_floor, 2
		
		rjmp button_pressed

	button_internal_floor_1_pressed:
		ldi floor_called, 1
		ldi external_or_internal_floor, 2
		
		rjmp button_pressed

	button_internal_floor_2_pressed:
		ldi floor_called, 2
		ldi external_or_internal_floor, 2
		
		rjmp button_pressed

	button_internal_floor_3_pressed:
		ldi floor_called, 3
		ldi external_or_internal_floor, 2
		
		rjmp button_pressed

	button_pressed:
		call Add_call
		call Delay_20ms
	/* ------Preparação dos atributos para executar a função (Add_call)------ */

	/* ---------------------------------------------------------------------- */

	/* -------------------------Início switch (estado)----------------------- */
	switch_state:
		cpi global_state, 1
		breq case_idle
		cpi global_state, 2
		breq case_elevator_ups
		cpi global_state, 3
		breq case_elevator_downs
		cpi global_state, 4
		breq case_new_floor
		cpi global_state, 5
		breq case_elevator_stops
		cpi global_state, 6
		breq case_buzzer_warning
		cpi global_state, 7
		breq case_holding_door
		cpi global_state, 8
		breq case_next_destiny

		case_idle:
			rjmp case_idle_longjump
		case_elevator_ups:
			rjmp case_elevator_ups_longjump
		case_elevator_downs:
			rjmp case_elevator_downs_longjump
		case_new_floor:
			rjmp case_new_floor_longjump
		case_elevator_stops:
			rjmp case_elevator_stops_longjump
		case_buzzer_warning:
			rjmp case_buzzer_warning_longjump
		case_holding_door:
			rjmp case_holding_door_longjump
		case_next_destiny:
			rjmp case_next_destiny_longjump

		/* -------------------------------CASOS----------------------------- */
		case_idle_longjump:
			// Verifica se o botão de abrir a porta está sendo pressionado
			sbic PIND, pin_button_open_door
			rjmp call_in_current_floor

			// Verifica o andar com maior prioridade
			call Get_high_priority_floor
			mov global_next_floor, next_destiny

			// Se não tiver chamada, ir para "no_call"
			cpi global_next_floor, -1
			breq no_call

			// Se a chamada for no andar atual, ir para "call_in_current_floor"
			cp global_next_floor, global_current_floor
			breq call_in_current_floor

			// Se a chamada for para subir, ir para "call_up"
			cp global_next_floor, global_current_floor
			brge call_up

			call_down:
				ldi global_direction, -1
				// Muda o estado para "case_elevator_downs"
				ldi global_state, 3
				// Reseta o contador
				call Clear_counter

				rjmp end_switch_state

			call_up:
				ldi global_direction, 1
				// Muda o estado para "case_elevator_ups"
				ldi global_state, 2
				// Reseta o contador
				call Clear_counter

				rjmp end_switch_state

			call_in_current_floor:
				ldi global_direction, 0
				// Muda o estado para "case_elevator_stops"
				ldi global_state, 5
				// Reseta o contador
				call Clear_counter

				rjmp end_switch_state

			no_call:
				ldi global_direction, 0

			rjmp end_switch_state

		case_elevator_ups_longjump:
			// Verifica se já se passaram 3s
			cpi global_counter, 3
			brne wait_elevator_ups_done

			// Incrementa o andar atual
			inc global_current_floor
			// Atualiza o valor do display
			call Update_display

			// Muda o estado para "case_new_floor"
			ldi global_state, 4
			// Reseta o contador
			call Clear_counter

			wait_elevator_ups_done:

			rjmp end_switch_state

		case_elevator_downs_longjump:
			// Verifica se já se passaram 3s
			cpi global_counter, 3
			brne wait_elevator_downs_done

			// Decrementa o andar atual
			dec global_current_floor
			// Atualiza o valor do display
			call Update_display

			// Muda o estado para "case_new_floor"
			ldi global_state, 4
			// Reseta o contador
			call Clear_counter

			wait_elevator_downs_done:

			rjmp end_switch_state

		case_new_floor_longjump:
			call Get_high_priority_floor

			// Verifica se esse é o andar com maior prioridade
			cp global_current_floor, next_destiny
			breq stop_elevator

			// Caso não seja o andar alvo, voltar para "case_idle"
			ldi global_state, 1

			rjmp end_switch_state

			stop_elevator:
				// Caso seja o andar alvo, vai para o próximo estado "case_elevator_stops"
				ldi global_state, 5

			rjmp end_switch_state

		case_elevator_stops_longjump:
			// Verifica se o botão de fechar a porta foi pressionado
			sbis PIND, pin_button_close_door
			rjmp skip_close_door_a

			// Muda o estado para "case_next_destiny"
			ldi global_state, 8
			// Reseta o contador
			call Clear_counter

			rjmp end_switch_state

			skip_close_door_a:
				// Ativa o led
				sbi PORTC, pin_led

				// Verifica se já se passaram 5s
				cpi global_counter, 5
				brne end_switch_state

				// Muda o estado para "case_buzzer_warning"
				ldi global_state, 6
				// Reseta o contador
				call Clear_counter

			rjmp end_switch_state

		case_buzzer_warning_longjump:
			// Verifica se o botão de fechar a porta foi pressionado
			sbis PIND, pin_button_close_door
			rjmp skip_close_door_b

			// Muda o estado para "case_next_destiny"
			ldi global_state, 8
			// Reseta o contador
			call Clear_counter

			rjmp end_switch_state

			skip_close_door_b:
				// Ativa o buzzer
				sbi PORTC, pin_buzzer

				// Verifica se já se passaram 5s
				cpi global_counter, 5
				brne end_switch_state

				// Muda o estado para "case_holding_door"
				ldi global_state, 7
				// Reseta o contador
				call Clear_counter

			rjmp end_switch_state

		case_holding_door_longjump:
			// Verifica se o botão de abrir a porta está sendo pressionado
			sbis PIND, pin_button_open_door
			// Caso o botão esteja sendo pressionado essa instrução é pulada
			rjmp close_door
		
			hold_door:
				// Aplica um delay para evitar o bounce
				call Delay_20ms

				rjmp end_switch_state

			close_door:
				// Muda o estado para "case_next_destiny"
				ldi global_state, 8

			rjmp end_switch_state

		case_next_destiny_longjump:
			// Desativa o buzzer e o led
			cbi PORTC, pin_buzzer
			cbi PORTC, pin_led

			// Remove a chamada da fila
			call Clear_call

			// Muda o estado para "case_idle"
			ldi global_state, 1

			rjmp end_switch_state
		end_switch_state:
		/* -------------------------------CASOS----------------------------- */

		.undef floor_called
		.undef external_or_internal_floor
		.undef next_destiny

	rjmp MAIN
