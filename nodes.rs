#[derive(Clone,Copy,Debug)]
pub struct nodes<'a>
{
    pub occupied:   bool,       //true if occupied
    pub direction:  &'a str,    //"N" "E" "S" "W" "no"
    pub head:       bool,       //true if the head
    pub tail:       bool,       //true if the tail
    pub food:       bool,        //true if food
    pub red:        u8,
    pub green:      u8,
    pub blue:       u8,
    pub position:   u8
}

impl nodes<'_>{
    pub fn new(occupied:bool,direction:&str,head:bool,tail:bool,food:bool,red:u8,green:u8,blue:u8, position:u8) -> nodes{
        nodes{
            occupied:occupied,
            direction:direction,
            head:head,
            tail:tail,
            food:food,
            red:red,
            green:green,
            blue:blue,
            position:position
        }
    }
}