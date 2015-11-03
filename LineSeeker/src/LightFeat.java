
public class LightFeat {
	
	public double meanLeft, meanRight, meanTot;
	
	public LightFeat() {
		init_all();
	}
	
	private void init_all() {
		meanLeft = 0.0;
		meanRight = 0.0;
		meanTot = 0.0;
	}
	
	public void compLeftRight(byte [][] luminanceFrame, int height, int width) {
		init_all();
		int halfWidth = width/2;
		double totPix = (height*width);
		double halfPix = totPix/2;
		for (int y=0; y<height; y++) {
			for (int x=0; x<halfWidth; x++) {
				meanLeft += (double) (luminanceFrame[y][x] & 0xFF); 
				meanTot += (double) (luminanceFrame[y][x] & 0xFF);
			}
			for (int x=halfWidth+1; x<width; x++) {
				meanRight += (double) (luminanceFrame[y][x] & 0xFF); 
				meanTot += (double) (luminanceFrame[y][x] & 0xFF); 
			}
		}
		meanLeft = meanLeft/halfPix;
		meanRight = meanRight/halfPix;
		meanTot = meanTot/totPix;
	}

}
