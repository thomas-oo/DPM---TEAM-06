package logging_data;

public class Filter {
	
	public int array []=null;
	
	public Filter(int array[]){
		this.array=array;
		
		shiting_array(array);
		
	}
	public void shiting_array(int [] array){
		if(array.length>4){
			for(int i=0; i<5;i++){
				array[i+1]=array[i];
			}
			median(array);
		}
		
	}
	
	  //sort the array, and return the median
    public int median(int[] a) {
        int temp;
        int asize = a.length;
        //sort the array in increasing order
        for (int i = 0; i < asize ; i++)
            for (int j = i+1; j < asize; j++)
                if (a[i] > a[j]) {
                    temp = a[i];
                    a[i] = a[j];
                    a[j] = temp;
                }
        //if it's odd
        if (asize%2 == 1)
            return a[asize/2];
        else
            return ((a[asize/2]+a[asize/2 - 1])/2);
    }

    public int [] getarray(){
    	return array;
    }
}
