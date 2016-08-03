package cs8803Lab1;

//only works for primitives
public class Tuple<X extends Comparable<X>,Y extends Comparable<Y>> implements Comparable<Tuple<X,Y>>{ 
    public final X x;    public final Y y; 
    public Tuple(X x, Y y) {    this.x = x;   this.y = y;  }
    public Tuple(Tuple<X,Y> _t) {    this.x = _t.x;   this.y = _t.y;  }
    public String toString() {      return "(" + x + "," + y + ")";  }
    @Override
    public boolean equals(Object _o) {  
    	if (_o == null) {return false;} 
    	if (_o == this) { return true; } 
    	if (!(_o instanceof Tuple)){ return false; } 
    	Tuple<X,Y> o = (Tuple<X,Y>) _o;  return o.x.equals(this.x) && o.y.equals(this.y);  }
    public int hashCode() { int result = 97 + ((x == null) ? 0 : x.hashCode()); return 97 * result + ((y == null) ? 0 : y.hashCode()); }
	@Override
	public int compareTo(Tuple<X,Y> o) {
		int i = ((Comparable<X>)x).compareTo(o.x);
	    if (i != 0) {return i;}
	    return ((Comparable<Y>)y).compareTo(o.y);
	}
	
}
