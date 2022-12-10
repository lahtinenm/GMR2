local function add(a, b)
    return a + b
end

local function sum(list)
    return Array.reduce(list, add, 0)
end

local function multiply(a, b)
    return a * b
end

local function product(list)
    return Array.reduce(list, multiply, 1)
end

local function round(value, numberOfDigitsAfterTheDecimalSeparator)
    if numberOfDigitsAfterTheDecimalSeparator == nil then
       numberOfDigitsAfterTheDecimalSeparator = 0
    end
    local a = 10^ numberOfDigitsAfterTheDecimalSeparator
    return math.floor(value * a + 0.5) / a
end

Math = {
    add = add,
    sum = sum,
    multiply = multiply,
    product = product,
    round = round
}
