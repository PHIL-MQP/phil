#include <Commands/Square.h>
#include <Commands/Forward.h>
#include <Commands/Turn.h>
#include <MoCapBot.h>

Square::Square() : CommandGroup("Square") {
	AddSequential(new Forward(1));
	AddSequential(new Turn(90));
	AddSequential(new Forward(1));
	AddSequential(new Turn(90));
	AddSequential(new Forward(1));
	AddSequential(new Turn(90));
	AddSequential(new Forward(1));
	AddSequential(new Turn(90));
}
