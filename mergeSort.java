import java.util.*;

public class Test
{
	// Main method
	public static void main(String args[])
	{
		int[] inputArr = {6, 3, 2, 1, 4};
		int[] sortedArr = mergeHelper(inputArr);
		
		System.out.println(sortedArr);
	}
	
	// Merge-Sort method for integer arrays
	public static int[] mergeSort(int[] inputArr)
	{
		if (inputArr.size() == 0)
		{
			return inputArr;
		}
		
		return mergeHelper(inputArr, 0, inputArr.size() - 1);
	}
	
	// This method handles the splitting and re-merging of the main array and its sub-arrays.
	private static int[] mergeHelper(int[] inputArr, int leftIndex, int rightIndex)
	{
		// Base case, array size of 1
		if (rightIndex == leftIndex)
		{
			int[] holder = new int[1];
			holder[0] = inputArr[leftIndex];
			return holder;
		}
		// We need to split and re-merge the arrays
		else
		{
			int middleIndex = (leftIndex + rightIndex) / 2;
			int[] leftArr = mergeHelper(inputArr, leftIndex, middleIndex);
			int[] rightArr = mergeHelper(inputArr, middleIndex + 1, rightIndex);
			int[] holder = new int[leftArr.size() + rightArr.size()];
			
			int leftPosition = 0;
			int rightPosition = 0;
			
			// Merging arrays back together
			while (leftPosition < leftArr.size() || rightPosition < rightArr.size())
			{
				// Both not empty, so compare
				if (leftPosition < leftArr.size() && rightPosition < rightArr.size())
				{
					// Left is larger/equal than right
					if (leftArr[leftPosition] - rightArr[rightPosition] >= 0)
					{
						holder[leftPosition + rightPosition] = leftArr[leftPosition];
						leftPosition++;
					}
					// Right is larger than left
					else
					{
						holder[leftPosition + rightPosition] = rightArr[rightPosition];
						rightPosition++;
					}
				}
				// Right array is empty
				else if (leftPosition < leftArr.size())
				{
					holder[leftPosition + rightPosition] = leftArr[leftPosition];
					leftPosition++;
				}
				// Left array is empty
				else
				{
					holder[leftPosition + rightPosition] = rightArr[rightPosition];
					rightPosition++;
				}
			}
			
			return holder;
		}
	}
		
}