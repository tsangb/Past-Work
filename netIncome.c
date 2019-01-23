#include <stdio.h>
#define ssTax 0.103
#define ssThreshold 65000.0
#define fedTax 0.28
#define fedThreshold 30000.0
#define fedBase 3500.0

/*
 * Given a yearly salary and a state tax, this program calculates the net income
 * you will make on a year by year basis for up to an input number of years.
 */
void netMoneyEarned(double salary, double stateTax, int years, double* monthlyIncome)
{
	double ssLeft = ssThreshold;
	for (int i = 0; i < years; i++)
	{
		monthlyIncome[i] = salary;
		
		if (ssLeft >= salary)
		{
			monthlyIncome[i] = monthlyIncome[i] - (salary * ssTax);
			ssLeft = ssLeft - salary;
		} else if (ssLeft != 0) {
			monthlyIncome[i] = monthlyIncome[i] - (ssLeft * ssTax);
			ssLeft = 0;
		}
		
		monthlyIncome[i] = monthlyIncome[i] - fedBase - (salary * stateTax);
		
		if (monthlyIncome[i] > fedThreshold)
		{
			monthlyIncome[i] = monthlyIncome[i] - ((monthlyIncome[i] - fedThreshold) * fedTax);
		}
	}
}

/*
 * Prompts the user for their salary and their state tax.
 * Then, calculates net income based upon input values while paying off the social security tax,
 * and all years thereafter (assuming no changes in salary).
 */
int main(int argc, char** argv)
{
	double salary;
	double stateTax;
	int years;
	
	printf("This program calculates your net income based on your income and the state tax.\n");
	printf("Please enter your monthly income: ");
	scanf("%lf", &salary);
	printf("Now please enter your state's tax rate, if any: ");
	scanf("%lf", &stateTax);
	printf("How many years on income do you want to monitor: ");
	scanf("%i", &years);
	double monthlyIncome[years];
	netMoneyEarned(salary, stateTax, years, monthlyIncome);
	for (int index = 0; index < years; index++)
	{
		printf("You will make %lf in year %i\n", monthlyIncome[index], index + 1);
	}
}


