function coin_toss()

    num = rand(1:10)
    #println("num is: ", num)
    
    if(num >= 6)
        return 1
    else
        return 0
    end
end

#result = coin_toss()
#println("result: ",result)
